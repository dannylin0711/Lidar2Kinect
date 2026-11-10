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

    func person(from observation: VNHumanBodyPoseObservation, quadrant: Int32) -> VisionPerson? {
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

/// One joint from Vision's 3D body pose: where it is in the buffer (normalised, origin top-left) and how far along the
/// optical axis (metres).
struct VisionJoint3D {
    var source: Int
    var u: Float
    var v: Float
    var depth: Float
    /// 2 = seen in the image, 1 = taken from the 3D model only
    var confidence: UInt8 = 2
}

/// 3D body pose from the camera image alone (no depth sensor). Vision estimates the joints in metres - with an assumed
/// body height when it has no depth - for one person.
///
/// Only two things are taken from Vision: each joint's position in the image and its distance along the optical axis.
/// The 3D point is then rebuilt with the camera's own intrinsics, so it does not depend on the axis conventions of
/// Vision's camera space, and it lines up with the camera image exactly.
final class VisionBody3DDetector {
    private let request = VNDetectHumanBodyPose3DRequest()
    private(set) var lastError: String?
    /// The body height Vision worked with (metres) and whether it measured it or assumed it.
    private(set) var lastHeight: Float = 0
    private(set) var lastHeightMeasured = false

    /// Vision's left / right are taken to be the person's own - to be confirmed on a device.
    private static let map: [(VNHumanBodyPose3DObservation.JointName, Int)] = [
        (.root, Int(L2K_SRC_PELVIS)), (.centerShoulder, Int(L2K_SRC_NECK)), (.centerHead, Int(L2K_SRC_HEAD)),
        (.leftShoulder, Int(L2K_SRC_SHOULDER_LEFT)), (.leftElbow, Int(L2K_SRC_ELBOW_LEFT)), (.leftWrist, Int(L2K_SRC_WRIST_LEFT)),
        (.rightShoulder, Int(L2K_SRC_SHOULDER_RIGHT)), (.rightElbow, Int(L2K_SRC_ELBOW_RIGHT)), (.rightWrist, Int(L2K_SRC_WRIST_RIGHT)),
        (.leftHip, Int(L2K_SRC_HIP_LEFT)), (.leftKnee, Int(L2K_SRC_KNEE_LEFT)), (.leftAnkle, Int(L2K_SRC_ANKLE_LEFT)),
        (.rightHip, Int(L2K_SRC_HIP_RIGHT)), (.rightKnee, Int(L2K_SRC_KNEE_RIGHT)), (.rightAnkle, Int(L2K_SRC_ANKLE_RIGHT)),
    ]

    /// The joints of the person Vision finds in `image` (empty if none), for a device rolled by `quadrant`.
    func detect(in image: CVPixelBuffer, quadrant: Int32) -> [VisionJoint3D] {
        let handler = VNImageRequestHandler(cvPixelBuffer: image, orientation: VisionOrientation.orientation(forQuadrant: quadrant), options: [:])
        do {
            try handler.perform([request])
            lastError = nil
        } catch {
            lastError = error.localizedDescription
            return []
        }
        guard let observation = request.results?.first else { return [] }
        lastHeight = observation.bodyHeight
        lastHeightMeasured = observation.heightEstimation == .measured
        return Self.joints(of: observation, quadrant: quadrant)
    }

    static func joints(of observation: VNHumanBodyPose3DObservation, quadrant: Int32) -> [VisionJoint3D] {
        var joints: [VisionJoint3D] = []
        for (name, source) in Self.map {
            guard let point = try? observation.pointInImage(name),
                  let position = try? observation.cameraRelativePosition(name) else { continue }
            let depth = abs(position.columns.3.z)
            guard depth.isFinite, depth > 0.05 else { continue }
            let b = VisionOrientation.bufferPoint(fromVision: point.location, quadrant: quadrant)
            joints.append(VisionJoint3D(source: source, u: b.u, v: b.v, depth: depth))
        }
        return joints
    }

    /// Place the joints in source camera space (+x image right, +y image up, the camera looks along -z).
    static func place(_ joints: [VisionJoint3D], camera: l2k_src_camera, into body: inout l2k_src_body) {
        for joint in joints {
            let x = (joint.u - camera.cxn) / camera.fxn * joint.depth
            let y = -(joint.v - camera.cyn) / camera.fyn * joint.depth
            body.set(joint.source, SIMD3(x, y, -joint.depth), confidence: joint.confidence)
        }
    }
}

/// Rebuilding a 3D pose from where the joints are in the picture, how long the bones are, and - for each bone -
/// whether it points towards the camera or away from it.
///
/// A single camera fixes the ray each joint lies on but not how far along it. Given the parent joint's 3D position
/// and the bone's length, the child joint is one of the (at most) two points of its ray at that distance from the
/// parent: one nearer the camera than the foot of the perpendicular, one farther. Which of the two is the only thing
/// taken from the 3D body model; the model's own depths, which drift when it does not fit the picture, are not used.
enum VisionKinematics {
    /// (joint, parent), from the trunk outwards, for a body rooted at the pelvis.
    static let tree: [(Int, Int)] = [
        (Int(L2K_SRC_NECK), Int(L2K_SRC_PELVIS)), (Int(L2K_SRC_HEAD), Int(L2K_SRC_NECK)),
        (Int(L2K_SRC_SHOULDER_LEFT), Int(L2K_SRC_NECK)), (Int(L2K_SRC_ELBOW_LEFT), Int(L2K_SRC_SHOULDER_LEFT)), (Int(L2K_SRC_WRIST_LEFT), Int(L2K_SRC_ELBOW_LEFT)),
        (Int(L2K_SRC_SHOULDER_RIGHT), Int(L2K_SRC_NECK)), (Int(L2K_SRC_ELBOW_RIGHT), Int(L2K_SRC_SHOULDER_RIGHT)), (Int(L2K_SRC_WRIST_RIGHT), Int(L2K_SRC_ELBOW_RIGHT)),
        (Int(L2K_SRC_HIP_LEFT), Int(L2K_SRC_PELVIS)), (Int(L2K_SRC_KNEE_LEFT), Int(L2K_SRC_HIP_LEFT)), (Int(L2K_SRC_ANKLE_LEFT), Int(L2K_SRC_KNEE_LEFT)),
        (Int(L2K_SRC_HIP_RIGHT), Int(L2K_SRC_PELVIS)), (Int(L2K_SRC_KNEE_RIGHT), Int(L2K_SRC_HIP_RIGHT)), (Int(L2K_SRC_ANKLE_RIGHT), Int(L2K_SRC_KNEE_RIGHT)),
    ]

    /// The camera-space point (x image right, y image up, looking along -z) of an image position at an optical-axis distance.
    static func point(u: Float, v: Float, depth: Float, camera: l2k_src_camera) -> SIMD3<Float> {
        SIMD3((u - camera.cxn) / camera.fxn * depth, -(v - camera.cyn) / camera.fyn * depth, -depth)
    }

    /// The child joint on the ray through (u, v), `length` away from `parent`. `away` picks the solution farther from
    /// the camera. When the ray passes farther than `length` from the parent (the picture disagrees with the bone
    /// length), the nearest point of the ray is used.
    static func child(parent: SIMD3<Float>, u: Float, v: Float, length: Float, away: Bool, camera: l2k_src_camera) -> SIMD3<Float> {
        let ray = simd_normalize(point(u: u, v: v, depth: 1, camera: camera))
        let along = simd_dot(ray, parent) // the foot of the perpendicular from the parent
        let discriminant = along * along - simd_length_squared(parent) + length * length
        let t = discriminant > 0 ? along + (away ? 1 : -1) * discriminant.squareRoot() : along
        return ray * max(t, 0.05)
    }

    /// Whether, in a model pose, the child is on the far side of the foot of the perpendicular from its parent.
    static func isAway(parent: SIMD3<Float>, child: SIMD3<Float>) -> Bool {
        let ray = simd_normalize(child)
        return simd_length(child) > simd_dot(ray, parent)
    }

    /// Re-place `joints` (their u, v kept, their depths replaced): the root keeps `rootDepth`; every other joint is
    /// solved from its parent with the bone `lengths` and the towards / `away` choice per joint. Joints whose parent
    /// or length is unknown keep the depth they came with.
    static func lift(_ joints: [VisionJoint3D], lengths: [Int: Float], away: [Int: Bool], camera: l2k_src_camera) -> [VisionJoint3D] {
        var bySource = Dictionary(uniqueKeysWithValues: joints.map { ($0.source, $0) })
        var points: [Int: SIMD3<Float>] = [:]
        let root = bySource[Int(L2K_SRC_PELVIS)] != nil ? Int(L2K_SRC_PELVIS) : Int(L2K_SRC_NECK)
        guard let rootJoint = bySource[root] else { return joints }
        points[root] = point(u: rootJoint.u, v: rootJoint.v, depth: rootJoint.depth, camera: camera)
        var edges = tree
        if root == Int(L2K_SRC_NECK) { edges.removeFirst() } // no pelvis: the neck is the root
        for (joint, parent) in edges {
            guard var j = bySource[joint] else { continue }
            if let p = points[parent], let length = lengths[joint] {
                let c = child(parent: p, u: j.u, v: j.v, length: length, away: away[joint] ?? false, camera: camera)
                j.depth = -c.z
                points[joint] = c
            } else {
                points[joint] = point(u: j.u, v: j.v, depth: j.depth, camera: camera)
            }
            bySource[joint] = j
        }
        return joints.map { bySource[$0.source] ?? $0 }
    }
}

/// Camera-only 3D pose that stays on the picture: where each joint is in the image comes from Vision's 2D body pose
/// (its 2D detector follows the body in the image closely), and how far away each joint is comes from Vision's 3D body
/// pose (which fits a whole-body model and can sit visibly off the picture when the body is close, leaning or cut off
/// - seen on an iPhone 17 Pro, 2026-11-10). A joint the 2D detector does not see is taken from the 3D model and
/// reported as estimated.
final class VisionHybridDetector {
    private let request2D = VNDetectHumanBodyPoseRequest()
    private let request3D = VNDetectHumanBodyPose3DRequest()
    private let detector2D = VisionBodyDetector()
    private(set) var lastError: String?
    private(set) var lastHeight: Float = 0
    private(set) var lastHeightMeasured = false
    /// How many joints of the last result came from the picture (2D) and how many from the 3D model alone.
    private(set) var lastSeen = 0, lastModelOnly = 0
    /// The source joints the picture showed in the last result.
    private(set) var lastSeenSources = Set<Int>()

    /// (joint, parent) from the trunk outwards.
    private static let hierarchy: [(Int, Int)] = [
        (Int(L2K_SRC_NECK), Int(L2K_SRC_PELVIS)), (Int(L2K_SRC_HEAD), Int(L2K_SRC_NECK)),
        (Int(L2K_SRC_SHOULDER_LEFT), Int(L2K_SRC_NECK)), (Int(L2K_SRC_ELBOW_LEFT), Int(L2K_SRC_SHOULDER_LEFT)), (Int(L2K_SRC_WRIST_LEFT), Int(L2K_SRC_ELBOW_LEFT)),
        (Int(L2K_SRC_SHOULDER_RIGHT), Int(L2K_SRC_NECK)), (Int(L2K_SRC_ELBOW_RIGHT), Int(L2K_SRC_SHOULDER_RIGHT)), (Int(L2K_SRC_WRIST_RIGHT), Int(L2K_SRC_ELBOW_RIGHT)),
        (Int(L2K_SRC_HIP_LEFT), Int(L2K_SRC_PELVIS)), (Int(L2K_SRC_KNEE_LEFT), Int(L2K_SRC_HIP_LEFT)), (Int(L2K_SRC_ANKLE_LEFT), Int(L2K_SRC_KNEE_LEFT)),
        (Int(L2K_SRC_HIP_RIGHT), Int(L2K_SRC_PELVIS)), (Int(L2K_SRC_KNEE_RIGHT), Int(L2K_SRC_HIP_RIGHT)), (Int(L2K_SRC_ANKLE_RIGHT), Int(L2K_SRC_KNEE_RIGHT)),
        (Int(L2K_SRC_PELVIS), Int(L2K_SRC_NECK)),
    ]

    /// Bone lengths of the person in view (metres, keyed by the child joint), averaged over time from the 3D model:
    /// a person's proportions are the stable part of what the model reports.
    private var boneLengths: [Int: Float] = [:]

    func detect(in image: CVPixelBuffer, quadrant: Int32, camera: l2k_src_camera) -> [VisionJoint3D] {
        let handler = VNImageRequestHandler(cvPixelBuffer: image, orientation: VisionOrientation.orientation(forQuadrant: quadrant), options: [:])
        do {
            try handler.perform([request2D, request3D])
            lastError = nil
        } catch {
            lastError = error.localizedDescription
            return []
        }
        guard let observation = request3D.results?.first else { return [] }
        lastHeight = observation.bodyHeight
        lastHeightMeasured = observation.heightEstimation == .measured
        let model = VisionBody3DDetector.joints(of: observation, quadrant: quadrant)
        guard !model.isEmpty else { return [] }
        let depth = Dictionary(uniqueKeysWithValues: model.map { ($0.source, $0.depth) })
        let modelRoot = model.first { $0.source == Int(L2K_SRC_PELVIS) } ?? model[0]

        // The 2D person that is the 3D request's person: the one whose centre is nearest to the model's root.
        let people = (request2D.results ?? []).compactMap { detector2D.person(from: $0, quadrant: quadrant) }
        let person = people.min { simd_distance($0.centre, SIMD2(modelRoot.u, modelRoot.v)) < simd_distance($1.centre, SIMD2(modelRoot.u, modelRoot.v)) }
        var joints: [VisionJoint3D] = []
        var seen = Set<Int>()
        if let person, simd_distance(person.centre, SIMD2(modelRoot.u, modelRoot.v)) < 0.25 {
            for joint in person.joints {
                guard let d = depth[joint.source] ?? depth[Int(L2K_SRC_PELVIS)] else { continue }
                joints.append(VisionJoint3D(source: joint.source, u: joint.u, v: joint.v, depth: d, confidence: 2))
                seen.insert(joint.source)
            }
        }
        lastSeen = joints.count
        lastSeenSources = seen
        // A joint the picture does not show keeps the model's offset from its parent joint, hung on the parent as
        // placed so far - so an unseen wrist follows the seen elbow instead of jumping to the model's own position.
        var placed = Dictionary(uniqueKeysWithValues: joints.map { ($0.source, $0) })
        let modelBySource = Dictionary(uniqueKeysWithValues: model.map { ($0.source, $0) })
        for (source, parent) in Self.hierarchy where placed[source] == nil {
            guard let m = modelBySource[source] else { continue }
            var joint = VisionJoint3D(source: source, u: m.u, v: m.v, depth: m.depth, confidence: 1)
            if let p = placed[parent], let mp = modelBySource[parent] {
                joint.u = p.u + (m.u - mp.u)
                joint.v = p.v + (m.v - mp.v)
            }
            placed[source] = joint
            joints.append(joint)
        }
        lastModelOnly = joints.count - lastSeen

        // Depths: from the picture and the bone lengths, with the model saying only which way each bone leans.
        var modelPoints: [Int: SIMD3<Float>] = [:]
        for m in model { modelPoints[m.source] = VisionKinematics.point(u: m.u, v: m.v, depth: m.depth, camera: camera) }
        var away: [Int: Bool] = [:]
        for (joint, parent) in VisionKinematics.tree {
            guard let c = modelPoints[joint], let p = modelPoints[parent] else { continue }
            let length = simd_distance(c, p)
            if length > 0.02, length < 1.2 {
                boneLengths[joint] = boneLengths[joint].map { $0 * 0.9 + length * 0.1 } ?? length
            }
            away[joint] = VisionKinematics.isAway(parent: p, child: c)
        }
        return VisionKinematics.lift(joints, lengths: boneLengths, away: away, camera: camera)
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

/// How often the picture showed each wrist and ankle over the last two seconds of frames - the joints a dance game
/// needs most and a detector loses first.
struct JointSeenRate {
    private static let watched: [(Int, String)] = [
        (Int(L2K_SRC_WRIST_LEFT), "L wrist"), (Int(L2K_SRC_WRIST_RIGHT), "R wrist"),
        (Int(L2K_SRC_ANKLE_LEFT), "L ankle"), (Int(L2K_SRC_ANKLE_RIGHT), "R ankle"),
    ]
    private var history: [Set<Int>] = []
    var window = 60

    /// Call once per processed frame that has a body, with the joints the picture showed.
    mutating func note(_ seen: Set<Int>) {
        history.append(seen)
        if history.count > window { history.removeFirst(history.count - window) }
    }

    var summary: String {
        guard !history.isEmpty else { return "Seen in the picture: no body yet" }
        return "Seen in the picture (last \(history.count) frames): " + Self.watched.map { source, name in
            "\(name) \(history.filter { $0.contains(source) }.count * 100 / history.count)%"
        }.joined(separator: ", ")
    }
}
