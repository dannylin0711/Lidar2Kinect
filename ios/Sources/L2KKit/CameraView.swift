// A camera-only source's view: which capture format shows the most, how far the preview must zoom out to show all of
// it, and which way the camera is mounted in the device. Pure maths, so it is tested on the Mac.
import Foundation
import L2KShared
import simd

/// What matters about one capture format when looking for the one with the whole view.
public struct L2KCameraFormat: Equatable {
    public var width: Int
    public var height: Int
    /// Horizontal field of view in degrees, as delivered (after geometric distortion correction when that is on).
    public var fieldOfView: Float
    /// The highest frame rate the format runs at.
    public var maxFrameRate: Double

    public init(width: Int, height: Int, fieldOfView: Float, maxFrameRate: Double) {
        self.width = width
        self.height = height
        self.fieldOfView = fieldOfView
        self.maxFrameRate = maxFrameRate
    }

    /// Half the view in tangent space (a pinhole camera: half the image width over the focal length).
    var tanHalfWidth: Float { tan(fieldOfView * .pi / 360) }
    var tanHalfHeight: Float { width > 0 ? tanHalfWidth * Float(height) / Float(width) : 0 }
    /// Vertical field of view in degrees.
    public var verticalFieldOfView: Float { 2 * atan(tanHalfHeight) * 180 / .pi }
    /// How much the format sees: the area of its view in tangent space.
    var viewArea: Float { tanHalfWidth * tanHalfHeight }
}

public enum L2KCameraView {
    /// The format that shows the most of the scene at `frameRate`: the largest view (a 4:3 format reads the whole
    /// sensor where a 16:9 one cuts the top and the bottom off). Frames wider than `maximumWidth` pixels are left out
    /// when smaller ones exist - 30 photo-sized frames a second are more than the detector can take. Among formats
    /// within 2 % of the largest view, the smallest one at least `minimumWidth` pixels wide (body detection does not
    /// need more), else the largest below that. nil for an empty list.
    public static func widestFormat(_ formats: [L2KCameraFormat], frameRate: Double = 30, minimumWidth: Int = 1280,
                                    maximumWidth: Int = 2048) -> Int? {
        var candidates = formats.indices.filter { formats[$0].maxFrameRate >= frameRate - 0.5 && formats[$0].viewArea > 0 }
        if candidates.isEmpty { candidates = formats.indices.filter { formats[$0].viewArea > 0 } }
        let modest = candidates.filter { formats[$0].width <= maximumWidth }
        if !modest.isEmpty { candidates = modest }
        guard let best = candidates.map({ formats[$0].viewArea }).max() else { return nil }
        let widest = candidates.filter { formats[$0].viewArea >= best * 0.98 }
        let enough = widest.filter { formats[$0].width >= minimumWidth }
        if let index = enough.min(by: { (formats[$0].width, formats[$0].height) < (formats[$1].width, formats[$1].height) }) { return index }
        return widest.max { (formats[$0].width, formats[$0].height) < (formats[$1].width, formats[$1].height) }
    }

    /// How many times the Kinect's field of view a virtual image must span, around the same centre, to show the whole
    /// camera image for a device rolled by `quadrant`. 1 when the camera sees no more than the Kinect.
    public static func zoomToFit(camera: l2k_src_camera, quadrant: Int32, limit: Float = 8) -> Float {
        var cam = camera
        func source(_ un: Float, _ vn: Float) -> SIMD2<Float> {
            var sx: Float = 0, sy: Float = 0
            l2k_virtual_to_source(&cam, quadrant, un, vn, &sx, &sy)
            return SIMD2(sx, sy)
        }
        // The mapping is affine: source = s0 + J * (virtual - 0.5).
        let s0 = source(0.5, 0.5)
        let jx = source(1.5, 0.5) - s0, jy = source(0.5, 1.5) - s0
        let det = jx.x * jy.y - jy.x * jx.y
        guard abs(det) > 1e-9 else { return 1 }
        var reach: Float = 0
        for corner in [SIMD2<Float>(0, 0), SIMD2(1, 0), SIMD2(0, 1), SIMD2(1, 1)] {
            let d = corner - s0
            let a = (d.x * jy.y - jy.x * d.y) / det, b = (jx.x * d.y - d.x * jx.y) / det
            reach = max(reach, abs(a), abs(b))
        }
        return min(max(2 * reach, 1), limit)
    }

    /// The roll quadrant (see `l2k_quadrant_from_up`) for the angle, in degrees clockwise, by which the system says the
    /// camera's frames have to be turned for a level horizon (`AVCaptureDevice.RotationCoordinator`).
    public static func quadrant(forLevelAngle degrees: Float) -> Int32 {
        let turns = Int32((degrees / 90).rounded()) & 3
        return (4 - turns) & 3
    }

    /// `up` with its in-image part turned by `quarterTurns` quarter turns (the part along the optical axis is kept).
    static func turned(_ up: SIMD3<Float>, quarterTurns: Int32) -> SIMD3<Float> {
        switch quarterTurns & 3 {
        case 1: return SIMD3(-up.y, up.x, up.z)
        case 2: return SIMD3(-up.x, -up.y, up.z)
        case 3: return SIMD3(up.y, -up.x, up.z)
        default: return up
        }
    }
}

/// Finds out which way round a camera sits in its device.
///
/// Gravity comes from the motion sensors in the device's axes; where "up" is in the camera's picture then depends on
/// how the sensor is mounted, which differs between cameras and devices (the front camera of one iPad is on a short
/// edge, of another on a long edge). The system knows: it reports the angle that makes that camera's frames level.
/// Whenever the device is clearly not lying flat, the roll worked out from gravity with the assumed mounting is
/// compared with the system's; a steady disagreement is taken as the mounting being turned by that many quarter turns.
public struct L2KCameraMounting {
    /// Quarter turns applied to the in-image part of "up" as worked out with the assumed mounting.
    public private(set) var quarterTurns: Int32 = 0
    /// Frames in which the roll could be compared, and in how many of them the two agreed (after the correction).
    public private(set) var compared = 0, agreed = 0
    private var candidate: Int32 = 0
    private var streak = 0
    /// A different mounting has to be seen this many times in a row before it is believed.
    public var framesToBelieve = 15
    /// False: only compare and count, never change the mounting (for a camera whose mounting is known to be right).
    public var adopts = true

    public init(adopts: Bool = true) { self.adopts = adopts }

    /// Give the up vector from gravity under the assumed mounting and, when known, the system's level angle (degrees
    /// clockwise). Returns the up vector to use.
    public mutating func correct(up: SIMD3<Float>, levelAngle: Float?) -> SIMD3<Float> {
        // Only a clearly rolled device says anything: not one lying flat, and not one near a diagonal.
        if let levelAngle, simd_length(SIMD2(up.x, up.y)) > 0.5, abs(abs(up.x) - abs(up.y)) > 0.4 {
            let wanted = L2KCameraView.quadrant(forLevelAngle: levelAngle)
            if let turns = (0..<4).map({ Int32($0) }).first(where: { k in
                let u = L2KCameraView.turned(up, quarterTurns: k)
                return Int32(l2k_quadrant_from_up(u.x, u.y)) == wanted
            }) {
                compared += 1
                if turns == quarterTurns {
                    agreed += 1
                    streak = 0
                } else {
                    if turns == candidate { streak += 1 } else { candidate = turns; streak = 1 }
                    if adopts, streak >= framesToBelieve {
                        quarterTurns = turns
                        streak = 0
                        compared = 0
                        agreed = 0
                    }
                }
            }
        }
        return L2KCameraView.turned(up, quarterTurns: quarterTurns)
    }

    public var summary: String {
        if compared == 0 { return "Camera mounting: not checked yet (the device has to stand up, not lie flat)" }
        let which = quarterTurns == 0 ? "as assumed" : "turned \(quarterTurns * 90)° from the assumed one (taken from the system)"
        return "Camera mounting: \(which); roll agrees with the system in \(agreed) of \(compared) checks"
    }
}
