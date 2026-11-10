// Capture source: ARKit body tracking on the rear camera (one body, 3D joints in metres).
// On LiDAR devices ARKit also gives scene depth and person segmentation when the configuration
// supports those frame semantics; what is actually available is checked at run time and reported.
#if os(iOS)
import ARKit
import Foundation
import L2KShared

public final class ARKitBodySource: NSObject, L2KSource, ARSessionDelegate {
    public let kind = UInt8(L2K_SOURCE_ARKIT_BODY)
    public let maxTracked = 1
    public let providesImages = true
    public var onFrame: ((L2KSourceFrame) -> Void)?

    public let session = ARSession()
    private let queue = DispatchQueue(label: "l2k.source.arkit")
    private let stateLock = NSLock()
    private var semantics: ARConfiguration.FrameSemantics = []
    private var lastStatus = "Starting"

    public static var isSupported: Bool { ARBodyTrackingConfiguration.isSupported }

    public var statusText: String {
        stateLock.lock()
        defer { stateLock.unlock() }
        return lastStatus
    }

    /// ARKit skeleton joint name for each source joint the mapper takes (L2K_SRC_* index).
    /// Hands are left out on purpose: ARKit does not track the wrist's rotation, so the mapper extends the forearm instead.
    private static let jointNames: [(Int, String)] = [
        (Int(L2K_SRC_PELVIS), "hips_joint"),
        (Int(L2K_SRC_NECK), "neck_1_joint"),
        (Int(L2K_SRC_SHOULDER_LEFT), "left_arm_joint"),
        (Int(L2K_SRC_ELBOW_LEFT), "left_forearm_joint"),
        (Int(L2K_SRC_WRIST_LEFT), "left_hand_joint"),
        (Int(L2K_SRC_SHOULDER_RIGHT), "right_arm_joint"),
        (Int(L2K_SRC_ELBOW_RIGHT), "right_forearm_joint"),
        (Int(L2K_SRC_WRIST_RIGHT), "right_hand_joint"),
        (Int(L2K_SRC_HIP_LEFT), "left_upLeg_joint"),
        (Int(L2K_SRC_KNEE_LEFT), "left_leg_joint"),
        (Int(L2K_SRC_ANKLE_LEFT), "left_foot_joint"),
        (Int(L2K_SRC_TOE_LEFT), "left_toes_joint"),
        (Int(L2K_SRC_HIP_RIGHT), "right_upLeg_joint"),
        (Int(L2K_SRC_KNEE_RIGHT), "right_leg_joint"),
        (Int(L2K_SRC_ANKLE_RIGHT), "right_foot_joint"),
        (Int(L2K_SRC_TOE_RIGHT), "right_toes_joint"),
    ]
    private static let headJointName = "head_joint"
    /// Distance from ARKit's head joint (where the head pivots on the neck) to the centre of the head, metres.
    /// An estimate to calibrate on device (docs/03 "Calibration").
    private static let headCentreOffset: Float = 0.09

    /// (source joint, ARKit joint index) for the names this OS version knows; `missing` lists the others.
    private let jointMap: [(Int, Int)]
    private let headIndex: Int?
    private let neckIndex: Int?
    public let missingJointNames: [String]

    public override init() {
        let definition = ARSkeletonDefinition.defaultBody3D
        func index(_ name: String) -> Int? {
            let i = definition.index(for: ARSkeleton.JointName(rawValue: name))
            return (i == NSNotFound || i < 0 || i >= definition.jointCount) ? nil : i
        }
        var map: [(Int, Int)] = []
        var missing: [String] = []
        for (source, name) in Self.jointNames {
            if let i = index(name) { map.append((source, i)) } else { missing.append(name) }
        }
        jointMap = map
        headIndex = index(Self.headJointName)
        neckIndex = index("neck_1_joint")
        if headIndex == nil { missing.append(Self.headJointName) }
        missingJointNames = missing
        super.init()
    }

    public func start() throws {
        guard ARBodyTrackingConfiguration.isSupported else {
            throw L2KSourceError.unsupported("ARKit body tracking is not supported on this device")
        }
        let configuration = ARBodyTrackingConfiguration()
        configuration.worldAlignment = .gravity
        configuration.automaticSkeletonScaleEstimationEnabled = true
        var wanted: ARConfiguration.FrameSemantics = []
        // Body detection itself is part of the configuration. Depth and segmentation are extras:
        // ask only for what this device and OS say they can do together with body tracking.
        for candidate in [ARConfiguration.FrameSemantics.personSegmentationWithDepth, .sceneDepth] where ARBodyTrackingConfiguration.supportsFrameSemantics(candidate) {
            wanted.insert(candidate)
        }
        configuration.frameSemantics = wanted
        stateLock.lock()
        semantics = wanted
        lastStatus = "Starting ARKit"
        stateLock.unlock()
        session.delegate = self
        session.delegateQueue = queue
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
    }

    public func stop() {
        session.pause()
    }

    // MARK: - ARSessionDelegate (on `queue`)

    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let cameraFromWorld = frame.camera.transform.inverse
        var bodies: [l2k_src_body] = []
        for case let anchor as ARBodyAnchor in frame.anchors where anchor.isTracked {
            bodies.append(body(from: anchor, cameraFromWorld: cameraFromWorld))
        }

        // World up is +y (gravity alignment): its coordinates in camera space are the y components of the camera axes.
        let t = frame.camera.transform
        let up = SIMD3<Float>(t.columns.0.y, t.columns.1.y, t.columns.2.y)
        let k = frame.camera.intrinsics
        let size = frame.camera.imageResolution
        let camera = l2k_src_camera(fxn: k.columns.0.x / Float(size.width), fyn: k.columns.1.y / Float(size.height),
                                    cxn: k.columns.2.x / Float(size.width), cyn: k.columns.2.y / Float(size.height))

        let depth = frame.sceneDepth?.depthMap ?? frame.estimatedDepthData
        let mask = frame.segmentationBuffer
        stateLock.lock()
        lastStatus = Self.describe(bodies: bodies.count, depth: depth != nil, mask: mask != nil,
                                   tracking: frame.camera.trackingState, missing: missingJointNames)
        stateLock.unlock()

        onFrame?(L2KSourceFrame(timestamp: frame.timestamp, bodies: bodies, upCamera: up, camera: camera,
                                depthMap: depth, segmentation: mask, image: frame.capturedImage))
    }

    public func session(_ session: ARSession, didFailWithError error: Error) {
        stateLock.lock()
        lastStatus = "ARKit failed: \(error.localizedDescription)"
        stateLock.unlock()
    }

    private func body(from anchor: ARBodyAnchor, cameraFromWorld: simd_float4x4) -> l2k_src_body {
        var body = l2k_src_body()
        // The anchor's UUID is stable while the body stays tracked: fold it into a nonzero 31-bit id.
        let uuid = anchor.identifier.uuid
        let folded = withUnsafeBytes(of: uuid) { raw in raw.reduce(UInt32(2166136261)) { ($0 ^ UInt32($1)) &* 16777619 } }
        body.tracking_id = (folded & 0x7FFF_FFFF) | 1

        let skeleton = anchor.skeleton
        let transforms = skeleton.jointModelTransforms
        let cameraFromBody = cameraFromWorld * anchor.transform
        func cameraPosition(_ index: Int) -> SIMD3<Float> {
            let p = cameraFromBody * transforms[index].columns.3
            return SIMD3(p.x, p.y, p.z)
        }
        for (source, index) in jointMap where index < transforms.count {
            body.set(source, cameraPosition(index), confidence: skeleton.isJointTracked(index) ? 2 : 1)
        }
        if let headIndex, headIndex < transforms.count {
            var head = cameraPosition(headIndex)
            if let neckIndex, neckIndex < transforms.count {
                let axis = head - cameraPosition(neckIndex)
                let length = simd_length(axis)
                if length > 1e-4 { head += axis / length * Self.headCentreOffset }
            }
            body.set(Int(L2K_SRC_HEAD), head, confidence: skeleton.isJointTracked(headIndex) ? 2 : 1)
        }
        return body
    }

    private static func describe(bodies: Int, depth: Bool, mask: Bool, tracking: ARCamera.TrackingState, missing: [String]) -> String {
        var parts = [bodies == 0 ? "No body in view" : "Body tracked"]
        parts.append(depth ? "depth on" : "no depth")
        parts.append(mask ? "segmentation on" : "no segmentation")
        if case .limited = tracking { parts.append("camera tracking limited") }
        if case .notAvailable = tracking { parts.append("camera tracking unavailable") }
        if !missing.isEmpty { parts.append("unknown joints: \(missing.joined(separator: ", "))") }
        return parts.joined(separator: " · ")
    }
}
#endif
