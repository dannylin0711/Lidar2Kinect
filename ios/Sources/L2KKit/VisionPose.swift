// Body pose and person mask from the camera image with the Vision framework.
// No ARKit here, so the coordinate handling is tested on the Mac.
import CoreVideo
import Foundation
import ImageIO
import L2KShared
import Vision

/// Vision works on an upright image. The camera buffer is in the sensor's native landscape, so the device's roll
/// (the shared library's quadrant) decides the orientation handed to Vision, and Vision's answers - normalised points
/// in the upright image, origin bottom-left - have to be turned back into buffer coordinates (origin top-left).
enum VisionOrientation {
    /// Quadrant: 0 = the buffer's up is up, 1 = the buffer's right is up, 2 = the buffer's down is up, 3 = the buffer's left is up.
    static func orientation(forQuadrant quadrant: Int32) -> CGImagePropertyOrientation {
        switch quadrant & 3 {
        case 1: return .left   // shown upright by turning the buffer a quarter turn anticlockwise
        case 2: return .down
        case 3: return .right  // ... clockwise (an iPhone held in portrait)
        default: return .up
        }
    }

    /// A Vision point (normalised, origin bottom-left, in the upright image) as a buffer position (normalised, origin top-left).
    static func bufferPoint(fromVision p: CGPoint, quadrant: Int32) -> (u: Float, v: Float) {
        let ox = Float(p.x), oy = 1 - Float(p.y) // upright image, origin top-left
        switch quadrant & 3 {
        case 1: return (1 - oy, ox)
        case 2: return (1 - ox, 1 - oy)
        case 3: return (oy, 1 - ox)
        default: return (ox, oy)
        }
    }
}

/// One person found in the image: the joints to lift, and where and how big they are (for following them over time).
struct VisionPerson {
    var joints: [DepthLiftJoint]
    var centre: SIMD2<Float> // buffer coordinates, normalised
    var size: Float          // neck-to-root distance in the buffer, normalised; 0 if unknown
}

/// 2D body pose of everyone in a camera buffer.
final class VisionBodyDetector {
    private let request = VNDetectHumanBodyPoseRequest()
    /// Points Vision is less sure of than this are ignored.
    var minimumConfidence: Float = 0.2
    private(set) var lastError: String?

    /// Vision joint -> source joint (L2K_SRC_*), with the distance from the body surface a depth sensor sees to the
    /// joint inside the body (metres; estimates, see docs/03 "Calibration"). Vision's left / right are taken to be the
    /// person's own left / right - to be confirmed on a device (the app colours the two sides differently).
    private static let map: [(VNHumanBodyPoseObservation.JointName, Int, Float)] = [
        (.root, Int(L2K_SRC_PELVIS), 0.11),
        (.neck, Int(L2K_SRC_NECK), 0.07),
        (.leftShoulder, Int(L2K_SRC_SHOULDER_LEFT), 0.06),
        (.leftElbow, Int(L2K_SRC_ELBOW_LEFT), 0.04),
        (.leftWrist, Int(L2K_SRC_WRIST_LEFT), 0.03),
        (.rightShoulder, Int(L2K_SRC_SHOULDER_RIGHT), 0.06),
        (.rightElbow, Int(L2K_SRC_ELBOW_RIGHT), 0.04),
        (.rightWrist, Int(L2K_SRC_WRIST_RIGHT), 0.03),
        (.leftHip, Int(L2K_SRC_HIP_LEFT), 0.09),
        (.leftKnee, Int(L2K_SRC_KNEE_LEFT), 0.055),
        (.leftAnkle, Int(L2K_SRC_ANKLE_LEFT), 0.04),
        (.rightHip, Int(L2K_SRC_HIP_RIGHT), 0.09),
        (.rightKnee, Int(L2K_SRC_KNEE_RIGHT), 0.055),
        (.rightAnkle, Int(L2K_SRC_ANKLE_RIGHT), 0.04),
    ]
    private static let headRadius: Float = 0.10

    /// Everyone Vision finds in `image`, for a device rolled by `quadrant`. Empty on failure (see `lastError`).
    func detect(in image: CVPixelBuffer, quadrant: Int32) -> [VisionPerson] {
        let handler = VNImageRequestHandler(cvPixelBuffer: image, orientation: VisionOrientation.orientation(forQuadrant: quadrant), options: [:])
        do {
            try handler.perform([request])
            lastError = nil
        } catch {
            lastError = error.localizedDescription
            return []
        }
        return (request.results ?? []).compactMap { person(from: $0, quadrant: quadrant) }
    }

    private func person(from observation: VNHumanBodyPoseObservation, quadrant: Int32) -> VisionPerson? {
        guard let points = try? observation.recognizedPoints(.all) else { return nil }
        func position(_ name: VNHumanBodyPoseObservation.JointName) -> SIMD2<Float>? {
            guard let p = points[name], p.confidence >= minimumConfidence else { return nil }
            let b = VisionOrientation.bufferPoint(fromVision: p.location, quadrant: quadrant)
            return SIMD2(b.u, b.v)
        }
        var joints: [DepthLiftJoint] = []
        for (name, source, radius) in Self.map {
            if let p = position(name) { joints.append(DepthLiftJoint(source: source, u: p.x, v: p.y, radius: radius)) }
        }
        // Vision has no "centre of the head": between the ears when both show, else the nose.
        if let l = position(.leftEar), let r = position(.rightEar) {
            joints.append(DepthLiftJoint(source: Int(L2K_SRC_HEAD), u: (l.x + r.x) / 2, v: (l.y + r.y) / 2, radius: Self.headRadius))
        } else if let nose = position(.nose) {
            joints.append(DepthLiftJoint(source: Int(L2K_SRC_HEAD), u: nose.x, v: nose.y, radius: Self.headRadius))
        }
        guard !joints.isEmpty else { return nil }
        let root = position(.root), neck = position(.neck)
        let centre = root ?? neck ?? SIMD2(joints[0].u, joints[0].v)
        let size: Float = (root != nil && neck != nil) ? simd_distance(root!, neck!) : 0
        return VisionPerson(joints: joints, centre: centre, size: size)
    }
}

/// Person mask of a camera buffer (kCVPixelFormatType_OneComponent8, in buffer coordinates), for devices where ARKit
/// gives no segmentation together with the depth map.
final class VisionPersonMask {
    private let request: VNGeneratePersonSegmentationRequest = {
        let request = VNGeneratePersonSegmentationRequest()
        request.qualityLevel = .fast
        request.outputPixelFormat = kCVPixelFormatType_OneComponent8
        return request
    }()
    private(set) var lastError: String?

    func mask(of image: CVPixelBuffer) -> CVPixelBuffer? {
        // Always in buffer orientation, so the mask stays registered to the depth map and the colour image.
        let handler = VNImageRequestHandler(cvPixelBuffer: image, orientation: .up, options: [:])
        do {
            try handler.perform([request])
            lastError = nil
            return request.results?.first?.pixelBuffer
        } catch {
            lastError = error.localizedDescription
            return nil
        }
    }
}

/// Keeps a stable id per person across frames by matching each detection to the nearest person of the previous frame.
struct VisionPersonTracker {
    private var tracks: [(id: UInt32, centre: SIMD2<Float>)] = []
    private var nextID: UInt32 = 1
    /// A detection further than this (normalised image units) from every known person is a new person.
    var maximumJump: Float = 0.25

    /// Ids for `people`, in the same order.
    mutating func assign(_ people: [VisionPerson]) -> [UInt32] {
        var ids = [UInt32](repeating: 0, count: people.count)
        var free = tracks
        // nearest pairs first
        var pairs: [(distance: Float, person: Int, track: Int)] = []
        for (i, person) in people.enumerated() {
            for (j, track) in free.enumerated() {
                pairs.append((simd_distance(person.centre, track.centre), i, j))
            }
        }
        var usedTracks = Set<Int>()
        for pair in pairs.sorted(by: { $0.distance < $1.distance }) where pair.distance <= maximumJump {
            guard ids[pair.person] == 0, !usedTracks.contains(pair.track) else { continue }
            ids[pair.person] = free[pair.track].id
            usedTracks.insert(pair.track)
        }
        free.removeAll()
        for i in people.indices where ids[i] == 0 {
            ids[i] = nextID
            nextID = nextID == UInt32.max ? 1 : nextID + 1
        }
        tracks = zip(ids, people).map { (id: $0.0, centre: $0.1.centre) }
        return ids
    }
}
