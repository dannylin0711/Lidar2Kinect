// Source frame -> Kinect frame: skeleton (shared C mapper), depth + player index image, colour image.
import CoreImage
import CoreVideo
import Foundation
import L2KShared
import simd

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
    /// For a local preview only: the camera image through a virtual camera `colorPreviewZoom` times wider than the
    /// Kinect's (640x480; the Kinect's own image is its central 1/zoom). nil unless a zoom above 1 was requested.
    public var colorPreview: CIImage?
    public var colorPreviewZoom: Float = 1
    /// Where the first body really stands: its level distance from the sensor in metres, before any play-area fit.
    public var realDistance: Float?
    /// The skeletons as the camera sees them, before the play-area fit or the normalising moved them: draw these
    /// over the camera picture. Equal to `skeleton` when neither is on.
    public var pictureSkeleton: l2k_skeleton_frame?
    /// The part of the camera picture `depth` and `color` show (see `L2KGameView`): the Kinect's own view unless the
    /// skeleton was moved by the play-area fit or the normalising.
    public var gameView = L2KGameView.identity
    /// The frame was (also) made for the local consumer: it carries the local-only images that were asked for.
    /// Frames in between, made only to be sent, do not - a preview must not show them.
    public var forLocal = false
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
    private var wantedKeepIdentity = true
    private var wantedFitPlayArea = false

    /// Slide a player who stands outside the game's play area (1.8-3.2 m from the sensor, 1.25 m either side) just
    /// inside it, and keep the reported floor consistent with the body. Off by default here. Safe to set from any thread.
    public var fitPlayArea: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedFitPlayArea }
        set { settingsLock.lock(); wantedFitPlayArea = newValue; settingsLock.unlock() }
    }

    private var wantedNormalize = false
    private var wantedNormalizePlayers = 1

    /// Report the players on Dance Evolution's own standing points, seen by a level sensor (`l2k_map_params.normalize`):
    /// one player 2.5 m in front of the sensor, two players 0.61 m either side of that. Replaces the fit when on.
    /// Safe to set from any thread.
    public var normalize: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedNormalize }
        set { settingsLock.lock(); wantedNormalize = newValue; settingsLock.unlock() }
    }

    /// How many players the game is told about when normalising (1 or 2). Safe to set from any thread.
    public var normalizePlayers: Int {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedNormalizePlayers }
        set { settingsLock.lock(); wantedNormalizePlayers = max(1, min(2, newValue)); settingsLock.unlock() }
    }

    /// The play area Dance Evolution accepts a player in (read from the game, docs/01 section 8), in metres.
    public static let playAreaNear: Float = 1.80, playAreaFar: Float = 3.20, playAreaHalfWidth: Float = 1.25

    /// A player who is lost and comes back gets the slot and tracking id they had (on by default). Off: a returning
    /// player is a new one after the half-second hold, as on a Kinect. Safe to set from any thread.
    public var keepIdentity: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedKeepIdentity }
        set { settingsLock.lock(); wantedKeepIdentity = newValue; settingsLock.unlock() }
    }

    /// Upper-body mode: track a player seen from about the waist up. The legs become a made-up standing pose
    /// (reported inferred) and every frame carries L2K_FRAME_UPPER_BODY. Safe to set from any thread.
    public var upperBodyMode: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedUpperBody }
        set { settingsLock.lock(); wantedUpperBody = newValue; settingsLock.unlock() }
    }

    private var wantedImagesFollow = true
    private var gameView = L2KGameView.identity
    private var gameViewSet = false

    /// Frame the depth and colour images on the skeleton that is sent (`L2KGameView`), so that a player who is
    /// reported somewhere else than the camera sees them - fitted into the play area, or normalised - is in the
    /// images where the skeleton says. On by default. Off: the images are always the middle of the camera picture.
    /// Safe to set from any thread.
    public var imagesFollowSkeleton: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedImagesFollow }
        set { settingsLock.lock(); wantedImagesFollow = newValue; settingsLock.unlock() }
    }

    public init() {
        l2k_mapper_init(&mapper, nil)
    }

    /// Where the joints of the tracked skeletons fall in a 320 × 240 Kinect image: of the skeletons that are sent and
    /// of the same skeletons as the camera sees them, joint for joint.
    private static func jointPixels(sent: l2k_skeleton_frame, picture: l2k_skeleton_frame) -> (sent: [SIMD2<Float>], picture: [SIMD2<Float>]) {
        var a: [SIMD2<Float>] = [], b: [SIMD2<Float>] = []
        let seen = Dictionary(uniqueKeysWithValues: picture.trackedSkeletons.map { ($0.slot, $0.skeleton) })
        for (slot, body) in sent.trackedSkeletons {
            guard let other = seen[slot] else { continue }
            let p = body.jointPositions, q = other.jointPositions, states = body.jointStates
            for index in 0..<min(p.count, q.count) where states[index] != UInt8(L2K_JOINT_NOT_TRACKED) {
                var u: Float = 0, v: Float = 0, s: Float = 0, t: Float = 0
                guard l2k_skeleton_to_depth_pixel(p[index], 320, 240, &u, &v) == 1,
                      l2k_skeleton_to_depth_pixel(q[index], 320, 240, &s, &t) == 1 else { continue }
                a.append(SIMD2(u, v))
                b.append(SIMD2(s, t))
            }
        }
        return (a, b)
    }

    /// The view for this frame's images: eased towards the fit so that it does not shake with the joints, and
    /// back to the Kinect's own view when nobody is tracked.
    private func updateGameView(sent: l2k_skeleton_frame, picture: l2k_skeleton_frame) -> L2KGameView {
        guard imagesFollowSkeleton else {
            gameView = .identity
            gameViewSet = false
            return gameView
        }
        let points = Self.jointPixels(sent: sent, picture: picture)
        if let fit = L2KGameView.fitting(sent: points.sent, picture: points.picture) {
            gameView = gameViewSet ? gameView.eased(towards: fit, by: 0.3) : fit
            gameViewSet = true
        } else {
            gameView = gameView.eased(towards: .identity, by: 0.03)
            if gameView.isIdentity { gameViewSet = false }
        }
        return gameView
    }

    /// Give a measured sensor height above the floor (metres); <= 0 returns to estimating it from the feet.
    public func setSensorHeight(_ metres: Float) {
        l2k_mapper_set_floor(&mapper, metres)
    }

    /// Build the Kinect frame for one source frame. Call from one queue only.
    /// - Parameter playersOnly: keep depth only on player pixels (L2K_DEPTH_MODE_PLAYERS).
    /// - Parameter colorPreviewFit: make `colorPreview` just wide enough to show the whole camera image, whatever
    ///   `colorPreviewZoom` says (no preview when the camera sees no more than the Kinect).
    public func process(_ source: L2KSourceFrame, kind: UInt8, wantDepth: Bool, wantColor: Bool, playersOnly: Bool,
                        wantDepthPreview: Bool = false, colorPreviewZoom: Float = 1, colorPreviewFit: Bool = false) -> L2KFrame {
        let q = updateQuadrant(up: source.upCamera)
        mapper.params.upper_body = upperBodyMode ? 1 : 0
        mapper.params.keep_identity = keepIdentity ? 1 : 0
        mapper.params.fit_play_area = fitPlayArea ? 1 : 0
        mapper.params.normalize = normalize ? 1 : 0
        mapper.params.norm_players = UInt8(normalizePlayers)
        var skeleton = l2k_skeleton_frame()
        let up = l2k_vec3(x: source.upCamera.x, y: source.upCamera.y, z: source.upCamera.z)
        source.bodies.withUnsafeBufferPointer { bodies in
            l2k_mapper_map(&mapper, bodies.baseAddress, Int32(bodies.count), up, q,
                           UInt64(source.timestamp * 1_000_000), kind, &skeleton)
        }
        var frame = L2KFrame(skeleton: skeleton, depth: nil, color: nil, quadrant: q)
        frame.realDistance = Self.realDistance(of: source.bodies.first, quadrant: q, up: skeleton.up)
        var picture = skeleton
        picture.skeletons = mapper.picture
        frame.pictureSkeleton = picture
        // The images go where the skeleton that is sent is (the same skeleton when nothing moved it: no change).
        frame.gameView = updateGameView(sent: skeleton, picture: picture)
        let gameCamera = frame.gameView.isIdentity ? source.camera : frame.gameView.camera(source.camera, quadrant: q)
        if wantDepth {
            frame.depth = depthImage(source, camera: gameCamera, skeleton: &skeleton, quadrant: q, playersOnly: playersOnly)
        }
        if wantColor, let image = source.image {
            frame.color = colorImage(image, camera: gameCamera, quadrant: q)
        }
        if wantDepthPreview {
            frame.depthPreview = depthPreview(source, quadrant: q)
        }
        let zoom = colorPreviewFit ? L2KCameraView.zoomToFit(camera: source.camera, quadrant: q) : colorPreviewZoom
        if zoom > 1, let image = source.image {
            frame.colorPreview = colorImage(image, camera: source.camera, quadrant: q, zoom: zoom)
            frame.colorPreviewZoom = zoom
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

    /// Level distance from the sensor of a source body (its pelvis, else its neck), nil if neither is known.
    static func realDistance(of body: l2k_src_body?, quadrant: Int32, up: l2k_vec3) -> Float? {
        guard let body else { return nil }
        let conf = withUnsafeBytes(of: body.conf) { Array($0) }
        let positions = withUnsafeBytes(of: body.p) { Array($0.bindMemory(to: l2k_vec3.self)) }
        guard let index = [Int(L2K_SRC_PELVIS), Int(L2K_SRC_NECK)].first(where: { conf[$0] != 0 }) else { return nil }
        let p = l2k_camera_to_kinect(positions[index], quadrant)
        let upV = SIMD3(up.x, up.y, up.z)
        var forward = SIMD3<Float>(0, 0, 1) - upV * upV.z
        guard simd_length(forward) > 1e-4 else { return nil }
        forward = simd_normalize(forward)
        return simd_dot(SIMD3(p.x, p.y, p.z), forward)
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

    private func depthImage(_ source: L2KSourceFrame, camera gameCamera: l2k_src_camera, skeleton: inout l2k_skeleton_frame, quadrant: Int32,
                            playersOnly: Bool) -> [UInt16] {
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
        var camera = gameCamera

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
    /// `zoom` > 1 widens the view around the same centre (for previews): the virtual image then spans
    /// zoom times the Kinect's field of view in tangent space.
    private func colorImage(_ buffer: CVPixelBuffer, camera: l2k_src_camera, quadrant: Int32, zoom: Float = 1) -> CIImage {
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
        let lo = 0.5 - zoom / 2, hi = 0.5 + zoom / 2 // the mapping is linear, so it extends outside 0...1
        let p00 = sourcePoint(lo, lo), p10 = sourcePoint(hi, lo), p01 = sourcePoint(lo, hi)
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
