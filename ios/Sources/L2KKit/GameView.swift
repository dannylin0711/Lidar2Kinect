// Which part of the camera picture the images sent to the game show.
//
// The game is told where the player is by the skeleton. When the skeleton is sent as the camera sees it, the Kinect
// images are simply the middle of the camera picture, at the Kinect's field of view. But the skeleton can be moved:
// the play-area fit slides a player who stands too near or too far to the edge of the game's area, and "Best position
// for Dance Evolution" puts the player 2.5 m in front of a level sensor wherever they really stand. The images then
// have to move with it, or the game gets a skeleton standing 2.5 m away together with a picture of a face - a player
// standing close to a wide camera fills the Kinect's narrow view (seen 2026-11-11 with the front camera).
//
// So the images are framed on the skeleton that is sent: scaled and shifted, inside the camera picture, until the
// player's joints in the picture lie where the sent skeleton's joints are in a Kinect image.
import Foundation
import L2KShared
import simd

public struct L2KGameView: Equatable {
    /// The images span this many times the Kinect's field of view (in tangent space). 1 = the Kinect's own.
    public var scale: Float = 1
    /// Their centre, measured from the centre of the Kinect's own image in that image's widths (x, to the right) and
    /// heights (y, downwards).
    public var dx: Float = 0, dy: Float = 0

    public static let identity = L2KGameView()

    public init(scale: Float = 1, dx: Float = 0, dy: Float = 0) {
        self.scale = scale
        self.dx = dx
        self.dy = dy
    }

    /// The view in which the points `sent` (where the skeleton that is sent projects into a 320 × 240 Kinect image)
    /// lie on the points `picture` (where the same joints are in the Kinect's own image of the camera picture):
    /// the scale from how far the points spread around their centre, the shift from the centres. nil when there
    /// are too few points or they do not spread.
    public static func fitting(sent: [SIMD2<Float>], picture: [SIMD2<Float>]) -> L2KGameView? {
        guard sent.count == picture.count, sent.count >= 4 else { return nil }
        let n = Float(sent.count)
        let sentCentre = sent.reduce(SIMD2<Float>(), +) / n, pictureCentre = picture.reduce(SIMD2<Float>(), +) / n
        let sentSpread = (sent.reduce(Float(0)) { $0 + simd_length_squared($1 - sentCentre) } / n).squareRoot()
        let pictureSpread = (picture.reduce(Float(0)) { $0 + simd_length_squared($1 - pictureCentre) } / n).squareRoot()
        guard sentSpread > 4, pictureSpread > 1 else { return nil }
        let scale = min(max(pictureSpread / sentSpread, 0.25), 6)
        let centre = SIMD2<Float>(160, 120)
        let shift = (pictureCentre - centre) - scale * (sentCentre - centre)
        return L2KGameView(scale: scale, dx: shift.x / 320, dy: shift.y / 240)
    }

    /// Part of the way from this view to `other`.
    public func eased(towards other: L2KGameView, by amount: Float) -> L2KGameView {
        L2KGameView(scale: scale + (other.scale - scale) * amount, dx: dx + (other.dx - dx) * amount, dy: dy + (other.dy - dy) * amount)
    }

    /// True when the view is the Kinect's own to within a pixel or so.
    public var isIdentity: Bool { abs(scale - 1) < 0.005 && abs(dx) < 0.003 && abs(dy) < 0.003 }

    /// The source camera to render the Kinect images with, so that they show this view: the virtual-to-source mapping
    /// is affine, so scaling and shifting the virtual image is the same as another focal length and principal point.
    public func camera(_ camera: l2k_src_camera, quadrant: Int32) -> l2k_src_camera {
        var cam = camera
        var cx: Float = 0, cy: Float = 0
        l2k_virtual_to_source(&cam, quadrant, 0.5 + dx, 0.5 + dy, &cx, &cy)
        return l2k_src_camera(fxn: camera.fxn * scale, fyn: camera.fyn * scale, cxn: cx, cyn: cy)
    }
}
