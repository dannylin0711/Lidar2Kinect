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
    /// Nose, eyes and ears where the picture shows them (L2K_SRC_NOSE ...): only for the 30-joint body, so kept apart
    /// from the skeleton's joints.
    var face: [DepthLiftJoint] = []
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
    /// Face points -> source joints, with the distance from the surface a depth sensor sees to the point (the nose and
    /// the ears are on the surface, an eye about a centimetre behind it; estimates).
    private static let faceMap: [(VNHumanBodyPoseObservation.JointName, Int, Float)] = [
        (.nose, Int(L2K_SRC_NOSE), 0), (.leftEye, Int(L2K_SRC_EYE_LEFT), 0.01), (.rightEye, Int(L2K_SRC_EYE_RIGHT), 0.01),
        (.leftEar, Int(L2K_SRC_EAR_LEFT), 0), (.rightEar, Int(L2K_SRC_EAR_RIGHT), 0),
    ]

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
        person(from: observation) { VisionOrientation.bufferPoint(fromVision: $0, quadrant: quadrant) }
    }

    /// `bufferPoint` turns a Vision point (normalised in the picture Vision was given, origin bottom-left) into a
    /// camera buffer position (normalised, origin top-left).
    func person(from observation: VNHumanBodyPoseObservation, bufferPoint: (CGPoint) -> (u: Float, v: Float)) -> VisionPerson? {
        guard let points = try? observation.recognizedPoints(.all) else { return nil }
        func position(_ name: VNHumanBodyPoseObservation.JointName) -> SIMD2<Float>? {
            guard let p = points[name], p.confidence >= minimumConfidence else { return nil }
            let b = bufferPoint(p.location)
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
        let face = Self.faceMap.compactMap { name, source, radius in
            position(name).map { DepthLiftJoint(source: source, u: $0.x, v: $0.y, radius: radius) }
        }
        return VisionPerson(joints: joints, centre: centre, size: size, face: face)
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
        joints(of: observation) { VisionOrientation.bufferPoint(fromVision: $0, quadrant: quadrant) }
    }

    /// `bufferPoint`: see `VisionBodyDetector.person(from:bufferPoint:)`.
    static func joints(of observation: VNHumanBodyPose3DObservation, bufferPoint: (CGPoint) -> (u: Float, v: Float)) -> [VisionJoint3D] {
        var joints: [VisionJoint3D] = []
        for (name, source) in Self.map {
            guard let point = try? observation.pointInImage(name),
                  let position = try? observation.cameraRelativePosition(name) else { continue }
            let depth = abs(position.columns.3.z)
            guard depth.isFinite, depth > 0.05 else { continue }
            let b = bufferPoint(point.location)
            joints.append(VisionJoint3D(source: source, u: b.u, v: b.v, depth: depth))
        }
        return joints
    }

    /// The focal length (pixels of a picture `height` pixels high) of the camera Vision placed the body with: the
    /// slope of the joints' heights in the picture against their height / distance in space. nil when the body does
    /// not span enough of the picture to tell. For checking that Vision used the intrinsics it was given.
    static func focalLength(of observation: VNHumanBodyPose3DObservation, pictureHeight height: Float) -> Float? {
        var xs: [Float] = [], ys: [Float] = []
        for name in observation.availableJointNames {
            guard let point = try? observation.pointInImage(name),
                  let position = try? observation.cameraRelativePosition(name) else { continue }
            let p = position.columns.3
            guard abs(p.z) > 0.05 else { continue }
            xs.append(p.y / p.z)
            ys.append(Float(point.location.y) * height)
        }
        guard xs.count >= 6 else { return nil }
        let n = Float(xs.count), mx = xs.reduce(0, +) / n, my = ys.reduce(0, +) / n
        var sxy: Float = 0, sxx: Float = 0
        for i in xs.indices {
            sxy += (xs[i] - mx) * (ys[i] - my)
            sxx += (xs[i] - mx) * (xs[i] - mx)
        }
        guard sxx > 0.01 else { return nil }
        return abs(sxy / sxx)
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
    /// length), or with `perpendicular`, the nearest point of the ray to the parent is used.
    static func child(parent: SIMD3<Float>, u: Float, v: Float, length: Float, away: Bool, camera: l2k_src_camera,
                      perpendicular: Bool = false) -> SIMD3<Float> {
        let ray = simd_normalize(point(u: u, v: v, depth: 1, camera: camera))
        let along = simd_dot(ray, parent) // the foot of the perpendicular from the parent
        let discriminant = along * along - simd_length_squared(parent) + length * length
        let t = discriminant > 0 && !perpendicular ? along + (away ? 1 : -1) * discriminant.squareRoot() : along
        return ray * max(t, 0.05)
    }

    /// Whether, in a model pose, the child is on the far side of the foot of the perpendicular from its parent.
    static func isAway(parent: SIMD3<Float>, child: SIMD3<Float>) -> Bool {
        let ray = simd_normalize(child)
        return simd_length(child) > simd_dot(ray, parent)
    }

    /// How a joint is placed from its parent.
    enum Placement: Equatable {
        /// On its ray at the bone's length: the far or the near solution (`away`).
        case length
        /// The foot of the perpendicular: the bone square to its ray, its length not kept.
        case perpendicular
        /// At the parent's distance from the camera: the bone parallel to the picture, its length not kept.
        case planar
        /// Planar, unless the picture shows the bone shorter than this fraction of its length: then `.length`.
        case planarUnlessShort(Float)
    }

    /// Re-place `joints` (their u, v kept, their depths replaced): the root keeps its depth; every other joint is
    /// placed from its parent by its `placement` (default `.length`, with the bone `lengths` and the towards / `away`
    /// choice per joint). Joints whose parent or length is unknown keep the depth they came with.
    static func lift(_ joints: [VisionJoint3D], lengths: [Int: Float], away: [Int: Bool], camera: l2k_src_camera,
                     placement: [Int: Placement] = [:]) -> [VisionJoint3D] {
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
                let onRay = { child(parent: p, u: j.u, v: j.v, length: length, away: away[joint] ?? false, camera: camera,
                                    perpendicular: placement[joint] == .perpendicular) }
                let flat = point(u: j.u, v: j.v, depth: -p.z, camera: camera)
                let c: SIMD3<Float>
                switch placement[joint] ?? .length {
                case .length, .perpendicular: c = onRay()
                case .planar: c = flat
                case .planarUnlessShort(let ratio): c = simd_distance(flat, p) < ratio * length ? onRay() : flat
                }
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
    private var request2D = VNDetectHumanBodyPoseRequest()
    private var request3D = VNDetectHumanBodyPose3DRequest()
    private let detector2D = VisionBodyDetector()
    private let canvas = VisionCanvas()
    private var wide: Bool?
    /// Give Vision the camera's intrinsics (true distances). Off: Vision assumes its own camera, as before 0.9.1.
    var givesIntrinsics = true
    /// The picture Vision was last given, and - when a body was found - the focal length it worked with against the
    /// one it was given (pixels).
    private(set) var lastLayout: VisionCanvasLayout?
    private(set) var lastFocalUsed: Float?, lastFocalGiven: Float = 0
    /// How long drawing the picture took (milliseconds).
    private(set) var lastDrawMs = 0.0
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

    /// How the limbs' depths are chosen (measured with the Dance Evolution port's sensor bench against rendered
    /// dancers with known poses and against videos of players: its docs/47; Lidar2Kinect docs/03 §7).
    /// - `model` (before 0.9.4): every bone towards or away from the camera as the 3D model has it. On the bench the
    ///   model's choice for forearms, thighs and shins was the wrong one more often than not.
    /// - `adaptive` (0.9.4): a limb bone lies parallel to the picture unless the picture shows it clearly shorter than
    ///   it is (arms < 0.8, legs < 0.75 of their length); a shortened upper arm, forearm or thigh points towards the
    ///   camera, a shortened shin away from it (a player facing the screen reaches towards it; a knee bends backwards).
    ///   The torso, shoulders and hips keep the model's choice.
    /// - `inverted`, `perpendicular`, `prior` (adaptive's towards / away without the parallel case), `planar` (limbs
    ///   always parallel): the alternatives measured on the bench.
    enum DepthRule { case model, inverted, perpendicular, prior, planar, adaptive }
    var depthRule = DepthRule.adaptive
    var armForeshortening: Float = 0.8, legForeshortening: Float = 0.75
    /// Where the bone lengths come from: the 3D model (before 0.9.4) or the picture's own spans (0.9.4).
    enum LengthSource { case model, picture }
    var lengthSource = LengthSource.picture
    private var pictureLengths: [Int: Float] = [:]

    /// What Vision reported for one picture, in camera buffer coordinates: the 3D model's joints and every person the 2D
    /// detector found. `combine` turns it into the joints that are sent; the two halves are apart so that a recorded
    /// observation can be combined again (the sensor bench of the Dance Evolution port, its docs/47).
    struct Observation {
        var model: [VisionJoint3D]
        var people: [VisionPerson]
    }

    func detect(in image: CVPixelBuffer, quadrant: Int32, camera: l2k_src_camera) -> [VisionJoint3D] {
        guard let observation = observe(image, quadrant: quadrant, camera: camera) else { return [] }
        return combine(observation, camera: camera)
    }

    /// Vision's two requests on one picture; nil when the 3D request finds nobody.
    func observe(_ image: CVPixelBuffer, quadrant: Int32, camera: l2k_src_camera) -> Observation? {
        // Vision sees an upright picture of a fixed size, never the camera buffer (see VisionCanvas.swift).
        let drawStart = ProcessInfo.processInfo.systemUptime
        guard let (picture, layout) = canvas.draw(image, quadrant: quadrant) else {
            lastError = "the picture for Vision could not be made"
            return nil
        }
        lastDrawMs = (ProcessInfo.processInfo.systemUptime - drawStart) * 1000
        lastLayout = layout
        if wide != layout.isWide {
            // A request keeps the shape of the first picture it saw: a new shape needs new requests.
            if wide != nil {
                request2D = VNDetectHumanBodyPoseRequest()
                request3D = VNDetectHumanBodyPose3DRequest()
            }
            wide = layout.isWide
        }
        var options: [VNImageOption: Any] = [:]
        var k = layout.intrinsics(of: camera)
        lastFocalGiven = k.columns.1.y
        if givesIntrinsics {
            options[.cameraIntrinsics] = Data(bytes: &k, count: MemoryLayout<matrix_float3x3>.size) as CFData
        }
        let handler = VNImageRequestHandler(cvPixelBuffer: picture, orientation: .up, options: options)
        do {
            try handler.perform([request2D, request3D])
            lastError = nil
        } catch {
            lastError = error.localizedDescription
            return nil
        }
        lastFocalUsed = nil
        guard let observation = request3D.results?.first else { return nil }
        lastHeight = observation.bodyHeight
        lastHeightMeasured = observation.heightEstimation == .measured
        lastFocalUsed = VisionBody3DDetector.focalLength(of: observation, pictureHeight: Float(layout.height))
        let model = VisionBody3DDetector.joints(of: observation, bufferPoint: layout.bufferPoint(fromVision:))
        let people = (request2D.results ?? []).compactMap { detector2D.person(from: $0, bufferPoint: layout.bufferPoint(fromVision:)) }
        return Observation(model: model, people: people)
    }

    /// The joints sent for one observation: where each joint is in the picture from the 2D detector, how far away from
    /// the bone lengths and the 3D model (see the type's comment). Keeps the bone lengths over time.
    func combine(_ observation: Observation, camera: l2k_src_camera) -> [VisionJoint3D] {
        let model = observation.model
        guard !model.isEmpty else { return [] }
        let depth = Dictionary(uniqueKeysWithValues: model.map { ($0.source, $0.depth) })
        let modelRoot = model.first { $0.source == Int(L2K_SRC_PELVIS) } ?? model[0]

        // The 2D person that is the 3D request's person: the one whose centre is nearest to the model's root.
        let people = observation.people
        let person = people.min { simd_distance($0.centre, SIMD2(modelRoot.u, modelRoot.v)) < simd_distance($1.centre, SIMD2(modelRoot.u, modelRoot.v)) }
        var joints: [VisionJoint3D] = []
        var seen = Set<Int>()
        var face: [DepthLiftJoint] = []
        if let person, simd_distance(person.centre, SIMD2(modelRoot.u, modelRoot.v)) < 0.25 {
            face = person.face
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

        // Depths: from the picture and the bone lengths, with the model saying which way each bone leans where the
        // depth rule leaves that to it.
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
            if depthRule == .inverted { away[joint]?.toggle() }
        }
        // the limbs: elbows and wrists hang on the arms, knees and ankles on the legs
        let arms = Set([L2K_SRC_ELBOW_LEFT, L2K_SRC_WRIST_LEFT, L2K_SRC_ELBOW_RIGHT, L2K_SRC_WRIST_RIGHT].map { Int($0) })
        let legs = Set([L2K_SRC_KNEE_LEFT, L2K_SRC_ANKLE_LEFT, L2K_SRC_KNEE_RIGHT, L2K_SRC_ANKLE_RIGHT].map { Int($0) })
        let shins: Set<Int> = [Int(L2K_SRC_ANKLE_LEFT), Int(L2K_SRC_ANKLE_RIGHT)]
        var placement: [Int: VisionKinematics.Placement] = [:]
        switch depthRule {
        case .model, .inverted: break
        case .perpendicular: for (j, _) in VisionKinematics.tree { placement[j] = .perpendicular }
        case .planar: for j in arms.union(legs) { placement[j] = .planar }
        case .prior, .adaptive:
            for j in arms.union(legs) {
                away[j] = shins.contains(j)
                if depthRule == .adaptive { placement[j] = .planarUnlessShort(arms.contains(j) ? armForeshortening : legForeshortening) }
            }
        }
        // Bone lengths as the picture shows them: the 2D detector's joints are not the 3D model's (on the bench its
        // thigh was ~15 % shorter, its shin up to ~15 % longer), so with the model's lengths a bone lying in the
        // picture plane is lifted out of it. The 90th percentile over time of the bone's span in the picture (both
        // joints seen, at the parent's model distance) is the length the bone shows when it lies flat.
        var lengths = boneLengths
        if lengthSource == .picture {
            let seenAt = Dictionary(uniqueKeysWithValues: joints.filter { $0.confidence == 2 }.map { ($0.source, $0) })
            for (joint, parent) in VisionKinematics.tree {
                guard let a = seenAt[joint], let b = seenAt[parent], let d = depth[parent], let model = boneLengths[joint] else { continue }
                let span = simd_distance(VisionKinematics.point(u: a.u, v: a.v, depth: d, camera: camera),
                                         VisionKinematics.point(u: b.u, v: b.v, depth: d, camera: camera))
                var l = pictureLengths[joint] ?? model
                let step = 0.01 * l
                l += span > l ? step * 0.9 : -step * 0.1
                pictureLengths[joint] = min(max(l, 0.5 * model), 1.6 * model)
            }
            for (j, l) in pictureLengths { lengths[j] = l }
        }
        let lifted = VisionKinematics.lift(joints, lengths: lengths, away: away, camera: camera, placement: placement)
        guard !face.isEmpty, let head = lifted.first(where: { $0.source == Int(L2K_SRC_HEAD) }) else { return lifted }
        return lifted + Self.placeFace(face, head: head, camera: camera)
    }

    /// Distance from the head centre to each face point on a typical head (metres): the 30-joint body's own
    /// estimates (docs/03 "The 30-joint body").
    static let faceDistance: [Int: Float] = [
        Int(L2K_SRC_NOSE): 0.087, Int(L2K_SRC_EYE_LEFT): 0.087, Int(L2K_SRC_EYE_RIGHT): 0.087,
        Int(L2K_SRC_EAR_LEFT): 0.076, Int(L2K_SRC_EAR_RIGHT): 0.076,
    ]

    /// The face points of a picture without depth, around the head as placed: each on its own ray, that far from the
    /// head centre - the nose and the eyes on the side towards the camera (a face the 2D detector sees is turned
    /// towards it), an ear at the point of its ray nearest the head centre (beside the head). Where the picture puts a
    /// point farther off the head than that, the nearest point of its ray is used.
    static func placeFace(_ face: [DepthLiftJoint], head: VisionJoint3D, camera: l2k_src_camera) -> [VisionJoint3D] {
        let centre = VisionKinematics.point(u: head.u, v: head.v, depth: head.depth, camera: camera)
        let ears: Set<Int> = [Int(L2K_SRC_EAR_LEFT), Int(L2K_SRC_EAR_RIGHT)]
        return face.map { point in
            let p = VisionKinematics.child(parent: centre, u: point.u, v: point.v, length: faceDistance[point.source] ?? 0.087,
                                           away: false, camera: camera, perpendicular: ears.contains(point.source))
            return VisionJoint3D(source: point.source, u: point.u, v: point.v, depth: -p.z, confidence: 2)
        }
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
