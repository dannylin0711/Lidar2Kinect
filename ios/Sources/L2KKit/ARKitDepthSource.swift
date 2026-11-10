// Capture source: the LiDAR depth map with ARKit's 2D body detection lifted into 3D.
//
// ARKit only delivers the LiDAR scene depth in a world-tracking session (its 3D body-tracking
// configuration does not), so this source runs world tracking with the frame semantics
// sceneDepth + personSegmentationWithDepth + bodyDetection - as many of them as the device
// supports together. Each 2D joint ARKit finds in the camera image is given the depth the LiDAR
// measured at that pixel, which yields a metric 3D skeleton; the depth map and the person mask
// become the Kinect depth + player index image.
//
// Not yet run against a real Kinect or tuned: the surface-to-joint offsets below are estimates
// (docs/03 "Calibration").
#if os(iOS)
import ARKit
import Foundation
import L2KShared

public final class ARKitDepthSource: NSObject, L2KSource, ARSessionDelegate {
    public let kind = UInt8(L2K_SOURCE_VISION_DEPTH)
    public let maxTracked = 1
    public let providesImages = true
    public var onFrame: ((L2KSourceFrame) -> Void)?

    public let session = ARSession()
    private let queue = DispatchQueue(label: "l2k.source.arkit.depth")
    private let stateLock = NSLock()
    private var lastStatus = "Starting"
    private var semantics: ARConfiguration.FrameSemantics = []

    /// The richest set of frame semantics this device runs together in a world-tracking session, nil if it cannot
    /// give both a body and a depth to lift it with.
    public static func bestSemantics() -> ARConfiguration.FrameSemantics? {
        let candidates: [ARConfiguration.FrameSemantics] = [
            [.sceneDepth, .personSegmentationWithDepth, .bodyDetection],
            [.sceneDepth, .bodyDetection],
            [.personSegmentationWithDepth, .bodyDetection], // no LiDAR: ARKit's estimated depth on person pixels
        ]
        return candidates.first { ARWorldTrackingConfiguration.supportsFrameSemantics($0) }
    }

    public static var isSupported: Bool {
        ARWorldTrackingConfiguration.isSupported && bestSemantics() != nil
    }

    public var statusText: String {
        stateLock.lock()
        defer { stateLock.unlock() }
        return lastStatus
    }

    /// One 2D joint -> source joint, with the distance from the body surface the LiDAR sees to the joint inside (metres).
    private struct JointSpec {
        let source: Int
        let name: String
        let radius: Float
    }

    private static let specs: [JointSpec] = [
        JointSpec(source: Int(L2K_SRC_PELVIS), name: "root", radius: 0.11),
        JointSpec(source: Int(L2K_SRC_NECK), name: "neck_1_joint", radius: 0.07),
        JointSpec(source: Int(L2K_SRC_HEAD), name: "head_joint", radius: 0.09),
        JointSpec(source: Int(L2K_SRC_SHOULDER_LEFT), name: "left_shoulder_1_joint", radius: 0.06),
        JointSpec(source: Int(L2K_SRC_ELBOW_LEFT), name: "left_forearm_joint", radius: 0.04),
        JointSpec(source: Int(L2K_SRC_WRIST_LEFT), name: "left_hand_joint", radius: 0.03),
        JointSpec(source: Int(L2K_SRC_SHOULDER_RIGHT), name: "right_shoulder_1_joint", radius: 0.06),
        JointSpec(source: Int(L2K_SRC_ELBOW_RIGHT), name: "right_forearm_joint", radius: 0.04),
        JointSpec(source: Int(L2K_SRC_WRIST_RIGHT), name: "right_hand_joint", radius: 0.03),
        JointSpec(source: Int(L2K_SRC_HIP_LEFT), name: "left_upLeg_joint", radius: 0.09),
        JointSpec(source: Int(L2K_SRC_KNEE_LEFT), name: "left_leg_joint", radius: 0.055),
        JointSpec(source: Int(L2K_SRC_ANKLE_LEFT), name: "left_foot_joint", radius: 0.04),
        JointSpec(source: Int(L2K_SRC_HIP_RIGHT), name: "right_upLeg_joint", radius: 0.09),
        JointSpec(source: Int(L2K_SRC_KNEE_RIGHT), name: "right_leg_joint", radius: 0.055),
        JointSpec(source: Int(L2K_SRC_ANKLE_RIGHT), name: "right_foot_joint", radius: 0.04),
    ]
    /// (spec, index in ARKit's 2D skeleton) for the joint names this OS version knows.
    private let jointMap: [(JointSpec, Int)]
    public let missingJointNames: [String]
    private var trackingID: UInt32 = 0
    private var bodyVisible = false

    public override init() {
        let definition = ARSkeletonDefinition.defaultBody2D
        var map: [(JointSpec, Int)] = []
        var missing: [String] = []
        for spec in Self.specs {
            let i = definition.index(for: ARSkeleton.JointName(rawValue: spec.name))
            if i == NSNotFound || i < 0 || i >= definition.jointCount { missing.append(spec.name) } else { map.append((spec, i)) }
        }
        jointMap = map
        missingJointNames = missing
        super.init()
    }

    public func start() throws {
        guard ARWorldTrackingConfiguration.isSupported, let wanted = Self.bestSemantics() else {
            throw L2KSourceError.unsupported("This device cannot give ARKit body detection together with a depth map")
        }
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
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
        let t = frame.camera.transform
        let up = SIMD3<Float>(t.columns.0.y, t.columns.1.y, t.columns.2.y)
        let k = frame.camera.intrinsics
        let size = frame.camera.imageResolution
        let camera = l2k_src_camera(fxn: k.columns.0.x / Float(size.width), fyn: k.columns.1.y / Float(size.height),
                                    cxn: k.columns.2.x / Float(size.width), cyn: k.columns.2.y / Float(size.height))

        // LiDAR scene depth when there is one, else ARKit's estimated depth of the person pixels.
        let depth = frame.sceneDepth?.depthMap ?? frame.estimatedDepthData
        let mask = frame.segmentationBuffer

        var bodies: [l2k_src_body] = []
        var lifted = 0
        if let detected = frame.detectedBody, let depth {
            if !bodyVisible {
                bodyVisible = true
                trackingID &+= 1 // a body that reappears is a new skeleton, as on a Kinect
                if trackingID == 0 { trackingID = 1 }
            }
            var body = l2k_src_body()
            body.tracking_id = trackingID
            lifted = lift(detected.skeleton, depth: depth, mask: mask, camera: camera, into: &body)
            if lifted > 0 { bodies.append(body) }
        } else {
            bodyVisible = false
        }

        stateLock.lock()
        lastStatus = describe(frame: frame, lifted: lifted, hasDepth: depth != nil, hasMask: mask != nil)
        stateLock.unlock()

        onFrame?(L2KSourceFrame(timestamp: frame.timestamp, bodies: bodies, upCamera: up, camera: camera,
                                depthMap: depth, segmentation: mask, image: frame.capturedImage))
    }

    public func session(_ session: ARSession, didFailWithError error: Error) {
        stateLock.lock()
        lastStatus = "ARKit failed: \(error.localizedDescription)"
        stateLock.unlock()
    }

    // MARK: - lifting 2D joints with the depth map

    /// Fill `body` from the 2D skeleton. Returns the number of joints that got a position.
    private func lift(_ skeleton: ARSkeleton2D, depth: CVPixelBuffer, mask: CVPixelBuffer?, camera: l2k_src_camera,
                      into body: inout l2k_src_body) -> Int {
        guard CVPixelBufferGetPixelFormatType(depth) == kCVPixelFormatType_DepthFloat32 else { return 0 }
        let usableMask = mask.flatMap { CVPixelBufferGetPixelFormatType($0) == kCVPixelFormatType_OneComponent8 ? $0 : nil }
        CVPixelBufferLockBaseAddress(depth, .readOnly)
        if let usableMask { CVPixelBufferLockBaseAddress(usableMask, .readOnly) }
        defer {
            CVPixelBufferUnlockBaseAddress(depth, .readOnly)
            if let usableMask { CVPixelBufferUnlockBaseAddress(usableMask, .readOnly) }
        }
        guard let depthBase = CVPixelBufferGetBaseAddress(depth) else { return 0 }
        var buffers = DepthLiftBuffers(depth: UnsafeRawPointer(depthBase), depthWidth: CVPixelBufferGetWidth(depth),
                                       depthHeight: CVPixelBufferGetHeight(depth), depthStride: CVPixelBufferGetBytesPerRow(depth))
        if let usableMask, let maskBase = CVPixelBufferGetBaseAddress(usableMask) {
            buffers.mask = UnsafeRawPointer(maskBase)
            buffers.maskWidth = CVPixelBufferGetWidth(usableMask)
            buffers.maskHeight = CVPixelBufferGetHeight(usableMask)
            buffers.maskStride = CVPixelBufferGetBytesPerRow(usableMask)
        }
        let landmarks = skeleton.jointLandmarks
        var joints: [DepthLiftJoint] = []
        for (spec, index) in jointMap where index < landmarks.count && skeleton.isJointTracked(index) {
            joints.append(DepthLiftJoint(source: spec.source, u: landmarks[index].x, v: landmarks[index].y, radius: spec.radius))
        }
        return DepthLift.lift(joints, buffers: buffers, camera: camera, into: &body)
    }

    private func describe(frame: ARFrame, lifted: Int, hasDepth: Bool, hasMask: Bool) -> String {
        var parts: [String] = []
        if frame.detectedBody == nil {
            parts.append("No body in view")
        } else {
            parts.append("Body detected, \(lifted)/\(jointMap.count) joints placed")
        }
        if frame.sceneDepth != nil {
            parts.append("LiDAR depth on")
        } else if hasDepth {
            parts.append("estimated depth only (no LiDAR depth)")
        } else {
            parts.append("no depth")
        }
        parts.append(hasMask ? "segmentation on" : "no segmentation")
        if case .limited = frame.camera.trackingState { parts.append("camera tracking limited") }
        if case .notAvailable = frame.camera.trackingState { parts.append("camera tracking unavailable") }
        if !missingJointNames.isEmpty { parts.append("unknown joints: \(missingJointNames.joined(separator: ", "))") }
        return parts.joined(separator: " · ")
    }
}

/// What this device's ARKit can do, for the app's diagnostics. Facts read from ARKit at run time.
public enum L2KDeviceSupport {
    public static func report() -> [(label: String, supported: Bool)] {
        [
            ("3D body tracking", ARBodyTrackingConfiguration.isSupported),
            ("3D body tracking + LiDAR depth", ARBodyTrackingConfiguration.supportsFrameSemantics(.sceneDepth)),
            ("3D body tracking + person segmentation", ARBodyTrackingConfiguration.supportsFrameSemantics(.personSegmentationWithDepth)),
            ("World tracking + LiDAR depth", ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)),
            ("World tracking + LiDAR depth + 2D body", ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .bodyDetection])),
            ("World tracking + LiDAR depth + 2D body + person segmentation",
             ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .personSegmentationWithDepth, .bodyDetection])),
        ]
    }
}
#endif
