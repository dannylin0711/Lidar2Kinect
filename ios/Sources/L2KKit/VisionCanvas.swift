// The picture handed to Vision's body pose: the camera buffer turned upright and fitted into one of two fixed sizes.
//
// Why not the camera buffer itself (seen on an iPhone 17 Pro, iOS 27.3, 2026-11-11, and on macOS 27.0):
// - Vision's 3D body pose pads its input to a 288 × 192 or 192 × 288 model. A square picture (the front camera's
//   whole-sensor format, 1280 × 1280) crashed inside that padding (`padImage` in AltruisticBodyPoseKit, EXC_BAD_ACCESS).
// - A 3D request that has seen a wide picture fails on a tall one and the other way round ("Invalid X-dimension
//   288/192" on the Mac, where a new request works again). On the iPhone the app twice printed "E5RT operation failed
//   ... Surface Object's width does not match with provided Surface Descriptor" and exited with code 1 while the
//   phone was being picked up; that is taken to be the same failure (not reproduced on purpose).
// - Without the camera's intrinsics Vision assumes a focal length of 0.76 × the picture's height (measured on the Mac:
//   the same body comes out at twice the distance when the focal length given is doubled), so with an ultra-wide
//   camera every distance was about twice too large. The intrinsics of this picture are known exactly and are given.
//
// So Vision always gets an upright 720 × 1280 or 1280 × 720 picture - the sizes it ran on for days - whatever the
// capture format is, and the requests are made anew when the shape changes.
import CoreGraphics
import CoreImage
import CoreVideo
import Foundation
import L2KShared
import simd

/// Where a camera buffer, turned upright for a device rolled by `quadrant`, sits in the picture given to Vision.
struct VisionCanvasLayout: Equatable {
    static let longSide = 1280, shortSide = 720

    /// The picture's size in pixels.
    let width: Int, height: Int
    let bufferWidth: Int, bufferHeight: Int
    let quadrant: Int32
    /// Camera buffer pixels to picture pixels.
    let scale: CGFloat
    /// The upright camera image inside the picture (pixels, origin top-left). The rest is black.
    let content: CGRect

    init(bufferWidth: Int, bufferHeight: Int, quadrant: Int32) {
        self.bufferWidth = bufferWidth
        self.bufferHeight = bufferHeight
        self.quadrant = quadrant & 3
        let turned = quadrant & 1 == 1
        let uprightWidth = CGFloat(turned ? bufferHeight : bufferWidth), uprightHeight = CGFloat(turned ? bufferWidth : bufferHeight)
        // A wide image gets the wide picture; a tall or a square one the tall picture (a standing person is tall).
        let wide = uprightWidth > uprightHeight
        width = wide ? Self.longSide : Self.shortSide
        height = wide ? Self.shortSide : Self.longSide
        scale = min(CGFloat(width) / max(uprightWidth, 1), CGFloat(height) / max(uprightHeight, 1))
        let w = uprightWidth * scale, h = uprightHeight * scale
        content = CGRect(x: (CGFloat(width) - w) / 2, y: (CGFloat(height) - h) / 2, width: w, height: h)
    }

    var isWide: Bool { width > height }

    /// A Vision point (normalised in the picture, origin bottom-left) as a camera buffer position (normalised, origin top-left).
    func bufferPoint(fromVision p: CGPoint) -> (u: Float, v: Float) {
        let x = (p.x * CGFloat(width) - content.minX) / content.width
        let fromTop = ((1 - p.y) * CGFloat(height) - content.minY) / content.height
        return VisionOrientation.bufferPoint(fromVision: CGPoint(x: x, y: 1 - fromTop), quadrant: quadrant)
    }

    /// A camera buffer position (normalised, origin top-left) in the picture (pixels, origin top-left).
    func picturePoint(fromBuffer u: Float, _ v: Float) -> CGPoint {
        let ox: Float, oy: Float // in the upright image, origin top-left
        switch quadrant & 3 {
        case 1: (ox, oy) = (v, 1 - u)
        case 2: (ox, oy) = (1 - u, 1 - v)
        case 3: (ox, oy) = (1 - v, u)
        default: (ox, oy) = (u, v)
        }
        return CGPoint(x: content.minX + CGFloat(ox) * content.width, y: content.minY + CGFloat(oy) * content.height)
    }

    /// The picture's pinhole camera for a buffer with `camera`, laid out as AVFoundation and Vision do
    /// (columns (fx 0 0) (0 fy 0) (cx cy 1), pixels, origin top-left).
    func intrinsics(of camera: l2k_src_camera) -> matrix_float3x3 {
        let fx = camera.fxn * Float(bufferWidth) * Float(scale), fy = camera.fyn * Float(bufferHeight) * Float(scale)
        let turned = quadrant & 1 == 1
        let centre = picturePoint(fromBuffer: camera.cxn, camera.cyn)
        return matrix_float3x3(columns: (SIMD3(turned ? fy : fx, 0, 0), SIMD3(0, turned ? fx : fy, 0), SIMD3(Float(centre.x), Float(centre.y), 1)))
    }
}

/// Draws camera buffers into the picture for Vision.
final class VisionCanvas {
    private let context = CIContext(options: [.cacheIntermediates: false])
    private var pictures: [Bool: CVPixelBuffer] = [:] // by `isWide`

    /// The picture for `buffer` on a device rolled by `quadrant`. nil if no picture could be made.
    func draw(_ buffer: CVPixelBuffer, quadrant: Int32) -> (picture: CVPixelBuffer, layout: VisionCanvasLayout)? {
        let layout = VisionCanvasLayout(bufferWidth: CVPixelBufferGetWidth(buffer), bufferHeight: CVPixelBufferGetHeight(buffer), quadrant: quadrant)
        guard layout.content.width >= 1, layout.content.height >= 1, let picture = picture(for: layout) else { return nil }
        var image = CIImage(cvPixelBuffer: buffer).oriented(VisionOrientation.orientation(forQuadrant: quadrant))
        image = image.transformed(by: CGAffineTransform(translationX: -image.extent.minX, y: -image.extent.minY))
        image = image.transformed(by: CGAffineTransform(scaleX: layout.scale, y: layout.scale))
        // Core Image's origin is bottom-left.
        image = image.transformed(by: CGAffineTransform(translationX: layout.content.minX,
                                                        y: CGFloat(layout.height) - layout.content.maxY))
        let bounds = CGRect(x: 0, y: 0, width: layout.width, height: layout.height)
        context.render(image.composited(over: CIImage(color: .black)).cropped(to: bounds), to: picture, bounds: bounds, colorSpace: nil)
        return (picture, layout)
    }

    private func picture(for layout: VisionCanvasLayout) -> CVPixelBuffer? {
        if let made = pictures[layout.isWide] { return made }
        var made: CVPixelBuffer?
        CVPixelBufferCreate(nil, layout.width, layout.height, kCVPixelFormatType_32BGRA,
                            [kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary] as CFDictionary, &made)
        pictures[layout.isWide] = made
        return made
    }
}
