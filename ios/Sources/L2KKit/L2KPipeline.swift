// Source frame -> Kinect frame: skeleton (shared C mapper), depth + player index image, colour image.
import CoreImage
import CoreVideo
import Foundation
import L2KShared

/// One frame of the virtual Kinect.
public struct L2KFrame {
    /// The skeleton frame (always present).
    public var skeleton: l2k_skeleton_frame
    /// 320x240 depth + player index, NUI packing; nil when not requested.
    public var depth: [UInt16]?
    /// 640x480 colour image registered to `depth` (extent 0,0,640,480); nil when not requested or the source has no camera image.
    public var color: CIImage?
    /// Device roll in quarter turns used for this frame.
    public var quadrant: Int32
    /// For a local preview only: what the depth sensor reports, seen through the virtual Kinect camera, before the
    /// Kinect range limits and the players-only setting are applied. nil when not requested or the source has no depth map.
    public var depthPreview: L2KDepthPreview?
}

/// The raw depth map resampled to the virtual Kinect image (320x240).
public struct L2KDepthPreview {
    /// Metres along the optical axis; 0 = the sensor reports nothing there.
    public var metres: [Float]
    /// Nonzero where the person mask says "person". Empty when the source has no mask.
    public var person: [UInt8]
}

public final class L2KPipeline {
    /// Shared Core Image context (GPU). Also used for the JPEG encoding and previews.
    public let ciContext = CIContext(options: [.cacheIntermediates: false])
    private let colorSpace = CGColorSpace(name: CGColorSpace.sRGB)!
    private var mapper = l2k_mapper()
    private var quadrant: Int32 = -1
    private let settingsLock = NSLock()
    private var wantedUpperBody = false

    /// Upper-body mode: track a player seen from about the waist up. The legs become a made-up standing pose
    /// (reported inferred) and every frame carries L2K_FRAME_UPPER_BODY. Safe to set from any thread.
    public var upperBodyMode: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedUpperBody }
        set { settingsLock.lock(); wantedUpperBody = newValue; settingsLock.unlock() }
    }

    public init() {
        l2k_mapper_init(&mapper, nil)
    }

    /// Give a measured sensor height above the floor (metres); <= 0 returns to estimating it from the feet.
    public func setSensorHeight(_ metres: Float) {
        l2k_mapper_set_floor(&mapper, metres)
    }

    /// Build the Kinect frame for one source frame. Call from one queue only.
    /// - Parameter playersOnly: keep depth only on player pixels (L2K_DEPTH_MODE_PLAYERS).
    public func process(_ source: L2KSourceFrame, kind: UInt8, wantDepth: Bool, wantColor: Bool, playersOnly: Bool,
                        wantDepthPreview: Bool = false) -> L2KFrame {
        let q = updateQuadrant(up: source.upCamera)
        mapper.params.upper_body = upperBodyMode ? 1 : 0
        var skeleton = l2k_skeleton_frame()
        let up = l2k_vec3(x: source.upCamera.x, y: source.upCamera.y, z: source.upCamera.z)
        source.bodies.withUnsafeBufferPointer { bodies in
            l2k_mapper_map(&mapper, bodies.baseAddress, Int32(bodies.count), up, q,
                           UInt64(source.timestamp * 1_000_000), kind, &skeleton)
        }
        var frame = L2KFrame(skeleton: skeleton, depth: nil, color: nil, quadrant: q)
        if wantDepth {
            frame.depth = depthImage(source, skeleton: &skeleton, quadrant: q, playersOnly: playersOnly)
        }
        if wantColor, let image = source.image {
            frame.color = colorImage(image, camera: source.camera, quadrant: q)
        }
        if wantDepthPreview {
            frame.depthPreview = depthPreview(source, quadrant: q)
        }
        return frame
    }

    /// JPEG of a frame's colour image.
    public func jpeg(_ color: CIImage, quality: Double) -> Data? {
        let options: [CIImageRepresentationOption: Any] = [
            CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): quality,
        ]
        return ciContext.jpegRepresentation(of: color, colorSpace: colorSpace, options: options)
    }

    // MARK: - roll

    /// The roll quadrant only changes when the device is clearly past the diagonal, so it does not flicker at 45 degrees.
    private func updateQuadrant(up: SIMD3<Float>) -> Int32 {
        let candidate = l2k_quadrant_from_up(up.x, up.y)
        if quadrant < 0 {
            quadrant = candidate
        } else if candidate != quadrant, abs(abs(up.x) - abs(up.y)) > 0.25 {
            quadrant = candidate
        }
        return quadrant
    }

    // MARK: - depth

    private func depthImage(_ source: L2KSourceFrame, skeleton: inout l2k_skeleton_frame, quadrant: Int32, playersOnly: Bool) -> [UInt16] {
        let width = Int32(L2K_DEPTH_WIDTH), height = Int32(L2K_DEPTH_HEIGHT)
        var pixels = [UInt16](repeating: 0, count: Int(width * height))
        let depthMap = source.depthMap.flatMap { CVPixelBufferGetPixelFormatType($0) == kCVPixelFormatType_DepthFloat32 ? $0 : nil }
        let mask = source.segmentation.flatMap { CVPixelBufferGetPixelFormatType($0) == kCVPixelFormatType_OneComponent8 ? $0 : nil }

        if depthMap == nil && mask == nil {
            // The source sees no pixels at all (synthetic, or a device without depth and segmentation): draw the skeletons.
            pixels.withUnsafeMutableBufferPointer { l2k_depth_rasterize(&skeleton, $0.baseAddress, width, height, 0) }
            return pixels
        }

        let tracked = skeleton.trackedSkeletons
        var src = l2k_depth_source()
        src.mask_threshold = 128
        src.player_index = UInt8((tracked.first?.slot ?? 0) + 1)
        src.players_only = (playersOnly && mask != nil) ? 1 : 0
        src.fallback_depth_m = tracked.first.map { $0.skeleton.position.z } ?? 0
        var camera = source.camera

        if let depthMap { CVPixelBufferLockBaseAddress(depthMap, .readOnly) }
        if let mask { CVPixelBufferLockBaseAddress(mask, .readOnly) }
        defer {
            if let depthMap { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
            if let mask { CVPixelBufferUnlockBaseAddress(mask, .readOnly) }
        }
        if let depthMap, let base = CVPixelBufferGetBaseAddress(depthMap) {
            src.depth_m = UnsafePointer(base.assumingMemoryBound(to: Float.self))
            src.depth_w = Int32(CVPixelBufferGetWidth(depthMap))
            src.depth_h = Int32(CVPixelBufferGetHeight(depthMap))
            src.depth_stride = CVPixelBufferGetBytesPerRow(depthMap)
        }
        if let mask, let base = CVPixelBufferGetBaseAddress(mask) {
            src.mask = UnsafePointer(base.assumingMemoryBound(to: UInt8.self))
            src.mask_w = Int32(CVPixelBufferGetWidth(mask))
            src.mask_h = Int32(CVPixelBufferGetHeight(mask))
            src.mask_stride = CVPixelBufferGetBytesPerRow(mask)
        }
        pixels.withUnsafeMutableBufferPointer { out in
            l2k_depth_render(&src, &camera, quadrant, out.baseAddress, width, height)
            if mask == nil {
                // Depth without segmentation: label the players by drawing the skeletons over the scene depth.
                l2k_depth_rasterize(&skeleton, out.baseAddress, width, height, 1)
            }
        }
        if mask != nil, tracked.count > 1 {
            relabelPlayers(&pixels, tracked: tracked)
        }
        return pixels
    }

    /// The person mask does not say who is who. With two tracked players, each player pixel goes to the player
    /// whose body centre is nearest in the image (by column).
    private func relabelPlayers(_ pixels: inout [UInt16], tracked: [(slot: Int, skeleton: l2k_skeleton)]) {
        let width = Int(L2K_DEPTH_WIDTH), height = Int(L2K_DEPTH_HEIGHT)
        var columns: [(u: Float, index: UInt16)] = []
        for player in tracked {
            var u: Float = 0, v: Float = 0
            if l2k_skeleton_to_depth_pixel(player.skeleton.position, Int32(width), Int32(height), &u, &v) == 1 {
                columns.append((u, UInt16(player.slot + 1)))
            }
        }
        guard columns.count > 1 else { return }
        let mask = UInt16(L2K_PLAYER_INDEX_MASK)
        for x in 0..<width {
            let nearest = columns.min { abs($0.u - Float(x)) < abs($1.u - Float(x)) }!.index
            for y in 0..<height {
                let i = y * width + x
                if pixels[i] & mask != 0 { pixels[i] = (pixels[i] & ~mask) | nearest }
            }
        }
    }

    /// The source's depth map (and mask) through the virtual Kinect camera, with nothing filtered out.
    private func depthPreview(_ source: L2KSourceFrame, quadrant: Int32) -> L2KDepthPreview? {
        guard let depthMap = source.depthMap, CVPixelBufferGetPixelFormatType(depthMap) == kCVPixelFormatType_DepthFloat32 else { return nil }
        let mask = source.segmentation.flatMap { CVPixelBufferGetPixelFormatType($0) == kCVPixelFormatType_OneComponent8 ? $0 : nil }
        let width = Int(L2K_DEPTH_WIDTH), height = Int(L2K_DEPTH_HEIGHT)
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        if let mask { CVPixelBufferLockBaseAddress(mask, .readOnly) }
        defer {
            CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
            if let mask { CVPixelBufferUnlockBaseAddress(mask, .readOnly) }
        }
        guard let depthBase = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let dw = CVPixelBufferGetWidth(depthMap), dh = CVPixelBufferGetHeight(depthMap), dstride = CVPixelBufferGetBytesPerRow(depthMap)
        let maskBase = mask.flatMap { CVPixelBufferGetBaseAddress($0) }
        let mw = mask.map { CVPixelBufferGetWidth($0) } ?? 0, mh = mask.map { CVPixelBufferGetHeight($0) } ?? 0
        let mstride = mask.map { CVPixelBufferGetBytesPerRow($0) } ?? 0
        var metres = [Float](repeating: 0, count: width * height)
        var person = [UInt8](repeating: 0, count: maskBase == nil ? 0 : width * height)
        var camera = source.camera
        for v in 0..<height {
            for u in 0..<width {
                var sx: Float = 0, sy: Float = 0
                l2k_virtual_to_source(&camera, quadrant, (Float(u) + 0.5) / Float(width), (Float(v) + 0.5) / Float(height), &sx, &sy)
                guard sx >= 0, sx < 1, sy >= 0, sy < 1 else { continue }
                let d = depthBase.advanced(by: Int(sy * Float(dh)) * dstride).assumingMemoryBound(to: Float.self)[Int(sx * Float(dw))]
                if d.isFinite, d > 0 { metres[v * width + u] = d }
                if let maskBase {
                    person[v * width + u] = maskBase.advanced(by: Int(sy * Float(mh)) * mstride + Int(sx * Float(mw)))
                        .assumingMemoryBound(to: UInt8.self).pointee >= 128 ? 1 : 0
                }
            }
        }
        return L2KDepthPreview(metres: metres, person: person)
    }

    // MARK: - colour

    /// Crop / rotate / mirror the camera image into the virtual Kinect's colour image.
    /// The two cameras share their centre and differ by a roll of quarter turns, so the mapping is affine.
    private func colorImage(_ buffer: CVPixelBuffer, camera: l2k_src_camera, quadrant: Int32) -> CIImage {
        let image = CIImage(cvPixelBuffer: buffer)
        let sw = CGFloat(CVPixelBufferGetWidth(buffer)), sh = CGFloat(CVPixelBufferGetHeight(buffer))
        let width = CGFloat(L2K_COLOR_WIDTH), height = CGFloat(L2K_COLOR_HEIGHT)
        var cam = camera

        // Where three corners of the virtual image fall in the source image (Core Image coordinates: origin bottom-left).
        func sourcePoint(_ un: Float, _ vn: Float) -> CGPoint {
            var sx: Float = 0, sy: Float = 0
            l2k_virtual_to_source(&cam, quadrant, un, vn, &sx, &sy)
            return CGPoint(x: CGFloat(sx) * sw, y: (1 - CGFloat(sy)) * sh)
        }
        let p00 = sourcePoint(0, 0), p10 = sourcePoint(1, 0), p01 = sourcePoint(0, 1)
        // Solve T(p00) = (0,h), T(p10) = (w,h), T(p01) = (0,0) for the affine T.
        let e1 = CGPoint(x: p10.x - p00.x, y: p10.y - p00.y), e2 = CGPoint(x: p01.x - p00.x, y: p01.y - p00.y)
        let det = e1.x * e2.y - e2.x * e1.y
        guard abs(det) > 1e-6 else { return image.cropped(to: .zero) }
        // A * [e1 e2] = [(w,0) (0,-h)]
        let a11 = width * e2.y / det, a12 = -width * e2.x / det
        let a21 = height * e1.y / det, a22 = -height * e1.x / det
        let tx = 0 - (a11 * p00.x + a12 * p00.y), ty = height - (a21 * p00.x + a22 * p00.y)
        let transform = CGAffineTransform(a: a11, b: a21, c: a12, d: a22, tx: tx, ty: ty)
        let extent = CGRect(x: 0, y: 0, width: width, height: height)
        // Parts of the virtual image the source camera does not see stay black.
        return image.transformed(by: transform).composited(over: CIImage(color: .black)).cropped(to: extent)
    }
}
