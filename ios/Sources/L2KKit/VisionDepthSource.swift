// Capture source: the LiDAR depth map from ARKit, with the body found by Vision in the camera image.
//
// Why this shape (measured on an iPad Pro M2 and an iPhone 17 Pro, 2026-11-10): ARKit gives the LiDAR
// scene depth only in a world-tracking session, and there it supports neither 2D body detection nor 3D
// body tracking alongside it. So ARKit supplies the camera image, its intrinsics, gravity and the depth
// map; Vision finds the 2D body joints of everyone in the image; each joint is given the depth the LiDAR
// measured at its pixel (DepthLift). The result is a metric 3D skeleton for up to two people, and the
// depth map with the person mask becomes the Kinect depth + player index image.
#if os(iOS)
import ARKit
import Foundation
import L2KShared

public final class VisionDepthSource: NSObject, L2KSource, ARSessionDelegate {
    public let kind = UInt8(L2K_SOURCE_VISION_DEPTH)
    public let maxTracked = 2
    public let providesImages = true
    public var onFrame: ((L2KSourceFrame) -> Void)?

    public let session = ARSession()
    private let queue = DispatchQueue(label: "l2k.source.vision.depth")
    private let detector = VisionBodyDetector()
    private let masker = VisionPersonMask()
    private var tracker = VisionPersonTracker()
    private var arkitMask = false
    private var jointsSeen = JointSeenRate()
    private var exposureLimited = false
    private var lastProcessed: TimeInterval = 0

    private let stateLock = NSLock()
    private var lastStatus = "Starting"
    private var lastDiagnostics: [String] = []
    // counters, on `queue`
    private var framesSeen = 0, framesProcessed = 0, framesStale = 0
    private var rateStart: TimeInterval = 0, rateSeen = 0, rateProcessed = 0
    private var seenRate = 0.0, processedRate = 0.0
    private var lastError: String?

    /// Frames are processed at most this often (a Kinect delivers 30 per second; ARKit's camera runs at 60). Set on
    /// the source's queue (`requestFrameRate`).
    public var processingRate: Double = 30

    /// ARKit delivers 60 camera frames a second; Vision on each of them is what limits the rate.
    public var maxFrameRate: Double { 60 }

    public func requestFrameRate(_ rate: Double) { processingRate = min(max(rate, 1), 60) }
    /// Take the person mask from Vision even where ARKit offers one together with the depth map.
    public var forceVisionMask = false

    public static var isSupported: Bool {
        ARWorldTrackingConfiguration.isSupported && ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
    }

    public var statusText: String {
        stateLock.lock()
        defer { stateLock.unlock() }
        return lastStatus
    }

    public var diagnostics: [String] {
        stateLock.lock()
        defer { stateLock.unlock() }
        return lastDiagnostics
    }

    public func start() throws {
        guard Self.isSupported else {
            throw L2KSourceError.unsupported("This device has no LiDAR depth in ARKit world tracking")
        }
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        var semantics: ARConfiguration.FrameSemantics = [.sceneDepth]
        // ARKit's own person mask when it can be had together with the depth map; Vision's otherwise.
        arkitMask = !forceVisionMask && ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .personSegmentationWithDepth])
        if arkitMask { semantics.insert(.personSegmentationWithDepth) }
        configuration.frameSemantics = semantics
        setStatus("Starting ARKit", diagnostics: ["Starting ARKit world tracking, person mask from \(arkitMask ? "ARKit" : "Vision")"])
        session.delegate = self
        session.delegateQueue = queue
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        // Short exposures, as in the ultra-wide source: less motion blur on fast hands and feet.
        if let camera = ARWorldTrackingConfiguration.configurableCaptureDeviceForPrimaryCamera, (try? camera.lockForConfiguration()) != nil {
            camera.activeMaxExposureDuration = CMTime(value: 1, timescale: UltraWideSource.shortestExposureDenominator)
            camera.unlockForConfiguration()
            exposureLimited = true
        }
    }

    public func stop() {
        session.pause()
    }

    // MARK: - ARSessionDelegate (on `queue`)

    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let now = ProcessInfo.processInfo.systemUptime
        framesSeen += 1
        rateSeen += 1
        // Keep to the processing rate, and never work on a frame that is already old: if Vision falls behind,
        // frames are skipped instead of queueing up.
        if now - frame.timestamp > 0.15 {
            framesStale += 1
            return
        }
        if lastProcessed > 0, frame.timestamp - lastProcessed < (1 / processingRate) * 0.75 { return }
        lastProcessed = frame.timestamp
        framesProcessed += 1
        rateProcessed += 1

        let t = frame.camera.transform
        let up = SIMD3<Float>(t.columns.0.y, t.columns.1.y, t.columns.2.y)
        let k = frame.camera.intrinsics
        let size = frame.camera.imageResolution
        let camera = l2k_src_camera(fxn: k.columns.0.x / Float(size.width), fyn: k.columns.1.y / Float(size.height),
                                    cxn: k.columns.2.x / Float(size.width), cyn: k.columns.2.y / Float(size.height))
        let quadrant = l2k_quadrant_from_up(up.x, up.y)
        let depth = frame.sceneDepth?.depthMap

        let visionStart = ProcessInfo.processInfo.systemUptime
        let people = detector.detect(in: frame.capturedImage, quadrant: quadrant)
        let mask = arkitMask ? frame.segmentationBuffer : masker.mask(of: frame.capturedImage)
        let visionMs = (ProcessInfo.processInfo.systemUptime - visionStart) * 1000

        // The two largest people, each with a stable id, lifted with the depth map.
        if let first = people.max(by: { $0.size < $1.size }) { jointsSeen.note(Set(first.joints.map(\.source))) }
        let ids = tracker.assign(people)
        let chosen = zip(people, ids).sorted { $0.0.size > $1.0.size }.prefix(maxTracked)
        var bodies: [l2k_src_body] = []
        var lifted: [Int] = []
        if let depth {
            for (person, id) in chosen {
                var body = l2k_src_body()
                body.tracking_id = id
                let n = Self.lift(person.joints + person.face, depth: depth, mask: mask, camera: camera, into: &body)
                lifted.append(n)
                if n > 0 { bodies.append(body) }
            }
        }

        updateStatus(frame: frame, now: now, quadrant: quadrant, depth: depth, mask: mask, people: people.count,
                     lifted: lifted, visionMs: visionMs)
        onFrame?(L2KSourceFrame(timestamp: frame.timestamp, bodies: bodies, upCamera: up, camera: camera,
                                depthMap: depth, segmentation: mask, image: frame.capturedImage))
    }

    public func session(_ session: ARSession, didFailWithError error: Error) {
        lastError = error.localizedDescription
        setStatus("ARKit failed: \(error.localizedDescription)", diagnostics: ["ARKit failed: \(error.localizedDescription)"])
    }

    // MARK: - lifting

    private static func lift(_ joints: [DepthLiftJoint], depth: CVPixelBuffer, mask: CVPixelBuffer?, camera: l2k_src_camera,
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
        return DepthLift.lift(joints, buffers: buffers, camera: camera, into: &body)
    }

    // MARK: - status and diagnostics

    private func setStatus(_ status: String, diagnostics: [String]) {
        stateLock.lock()
        lastStatus = status
        lastDiagnostics = diagnostics
        stateLock.unlock()
    }

    private func updateStatus(frame: ARFrame, now: TimeInterval, quadrant: Int32, depth: CVPixelBuffer?, mask: CVPixelBuffer?,
                              people: Int, lifted: [Int], visionMs: Double) {
        if rateStart == 0 { rateStart = now }
        if now - rateStart >= 1 {
            seenRate = Double(rateSeen) / (now - rateStart)
            processedRate = Double(rateProcessed) / (now - rateStart)
            rateStart = now
            rateSeen = 0
            rateProcessed = 0
        }
        var parts: [String] = []
        if people == 0 {
            parts.append("No body in view")
        } else {
            parts.append("\(people) \(people == 1 ? "person" : "people") seen, \(lifted.filter { $0 > 0 }.count) placed")
        }
        parts.append(depth != nil ? "LiDAR depth on" : "no depth")
        parts.append(mask != nil ? "person mask on" : "no person mask")
        if case .limited = frame.camera.trackingState { parts.append("camera tracking limited") }
        if case .notAvailable = frame.camera.trackingState { parts.append("camera tracking unavailable") }

        // The detailed lines cost a pass over the buffers: refresh them a few times a second only.
        guard framesProcessed % 8 == 1 else {
            stateLock.lock()
            lastStatus = parts.joined(separator: " · ")
            stateLock.unlock()
            return
        }
        var lines: [String] = []
        lines.append(String(format: "ARKit frames %d (%.0f/s), processed %d (%.0f/s), skipped as late %d",
                            framesSeen, seenRate, framesProcessed, processedRate, framesStale))
        lines.append("Image \(Int(frame.camera.imageResolution.width))×\(Int(frame.camera.imageResolution.height)), roll quadrant \(quadrant)")
        if let depth {
            lines.append(Self.describeDepth(depth))
        } else {
            lines.append("Depth map: none in this frame (sceneDepth \(frame.sceneDepth == nil ? "nil" : "set"), smoothed \(frame.smoothedSceneDepth == nil ? "nil" : "set"), estimated \(frame.estimatedDepthData == nil ? "nil" : "set"))")
        }
        lines.append("Camera tracking: \(Self.describe(frame.camera.trackingState)), semantics asked: \(session.configuration?.frameSemantics.rawValue ?? 0)")
        if let mask {
            lines.append("Person mask (\(arkitMask ? "ARKit" : "Vision")): " + Self.describeMask(mask))
        } else {
            lines.append("Person mask (\(arkitMask ? "ARKit" : "Vision")): none" + (masker.lastError.map { " — \($0)" } ?? ""))
        }
        lines.append(String(format: "Vision: %d people, %.0f ms", people, visionMs) + (detector.lastError.map { " — \($0)" } ?? ""))
        if !lifted.isEmpty { lines.append("Joints placed per person: " + lifted.map(String.init).joined(separator: ", ") + " of 15") }
        lines.append(jointsSeen.summary)
        lines.append("Exposure limited to 1/\(UltraWideSource.shortestExposureDenominator) s: \(exposureLimited ? "yes" : "no")")
        if let lastError { lines.append("Last ARKit error: \(lastError)") }
        setStatus(parts.joined(separator: " · "), diagnostics: lines)
    }

    private static func describe(_ state: ARCamera.TrackingState) -> String {
        switch state {
        case .normal: return "normal"
        case .notAvailable: return "not available"
        case .limited(let reason):
            switch reason {
            case .initializing: return "limited (initializing)"
            case .excessiveMotion: return "limited (excessive motion)"
            case .insufficientFeatures: return "limited (insufficient features)"
            case .relocalizing: return "limited (relocalizing)"
            @unknown default: return "limited"
            }
        }
    }

    private static func describeDepth(_ depth: CVPixelBuffer) -> String {
        guard CVPixelBufferGetPixelFormatType(depth) == kCVPixelFormatType_DepthFloat32 else { return "Depth map: unexpected pixel format" }
        CVPixelBufferLockBaseAddress(depth, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depth, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(depth) else { return "Depth map: no data" }
        let w = CVPixelBufferGetWidth(depth), h = CVPixelBufferGetHeight(depth), stride = CVPixelBufferGetBytesPerRow(depth)
        var valid = 0, nearest = Float.greatestFiniteMagnitude, farthest: Float = 0
        for y in 0..<h {
            let row = base.advanced(by: y * stride).assumingMemoryBound(to: Float.self)
            for x in 0..<w where row[x].isFinite && row[x] > 0 {
                valid += 1
                nearest = min(nearest, row[x])
                farthest = max(farthest, row[x])
            }
        }
        let centre = base.advanced(by: (h / 2) * stride).assumingMemoryBound(to: Float.self)[w / 2]
        guard valid > 0 else { return "Depth map \(w)×\(h): no valid pixel" }
        return String(format: "Depth map %d×%d: centre %.2f m, range %.2f–%.2f m, %d%% valid", w, h, centre, nearest, farthest, valid * 100 / (w * h))
    }

    private static func describeMask(_ mask: CVPixelBuffer) -> String {
        guard CVPixelBufferGetPixelFormatType(mask) == kCVPixelFormatType_OneComponent8 else { return "unexpected pixel format" }
        CVPixelBufferLockBaseAddress(mask, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(mask, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(mask) else { return "no data" }
        let w = CVPixelBufferGetWidth(mask), h = CVPixelBufferGetHeight(mask), stride = CVPixelBufferGetBytesPerRow(mask)
        var person = 0
        for y in 0..<h {
            let row = base.advanced(by: y * stride).assumingMemoryBound(to: UInt8.self)
            for x in 0..<w where row[x] >= 128 { person += 1 }
        }
        return String(format: "%d×%d, %.1f%% person pixels", w, h, Double(person) * 100 / Double(w * h))
    }
}

/// What this device's ARKit can do, for the app's diagnostics. Facts read from ARKit at run time.
public enum L2KDeviceSupport {
    /// The camera formats ARKit offers for world tracking and for body tracking, one line each.
    public static func videoFormats() -> [String] {
        func describe(_ label: String, _ formats: [ARConfiguration.VideoFormat]) -> [String] {
            formats.map { f in
                "format \(label): \(f.captureDeviceType.rawValue) \(Int(f.imageResolution.width))x\(Int(f.imageResolution.height)) @\(f.framesPerSecond)"
            }
        }
        return describe("world", ARWorldTrackingConfiguration.supportedVideoFormats)
            + describe("body", ARBodyTrackingConfiguration.supportedVideoFormats)
    }

    public static func report() -> [(label: String, supported: Bool)] {
        [
            ("World tracking + LiDAR depth", ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)),
            ("World tracking + LiDAR depth + person mask",
             ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .personSegmentationWithDepth])),
            ("World tracking + LiDAR depth + 2D body", ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .bodyDetection])),
            ("3D body tracking", ARBodyTrackingConfiguration.isSupported),
            ("3D body tracking + LiDAR depth", ARBodyTrackingConfiguration.supportsFrameSemantics(.sceneDepth)),
            ("3D body tracking + person mask", ARBodyTrackingConfiguration.supportsFrameSemantics(.personSegmentationWithDepth)),
        ]
    }
}
#endif
