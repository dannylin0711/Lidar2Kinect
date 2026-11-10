// Capture source: the rear ultra-wide camera alone, with Vision's 3D body pose. No LiDAR.
//
// The LiDAR only covers the normal wide camera's view (and ARKit offers only that camera - read from its
// supported video formats on an iPhone 17 Pro), so a wider view means doing without measured depth. The
// ultra-wide camera sees a whole standing person from about a metre away. Vision estimates the 3D pose of
// one person from the image; the depth image is the person mask at the body's distance, or the figure drawn
// from the skeleton.
//
// Distances are Vision's estimate (it assumes a body height when it cannot measure one), so the skeleton is
// right in shape and direction but only roughly in scale. Dance Evolution judges limb directions, not size.
#if os(iOS)
import AVFoundation
import CoreMotion
import Foundation
import L2KShared

public final class UltraWideSource: NSObject, L2KSource, AVCaptureVideoDataOutputSampleBufferDelegate {
    public let kind = UInt8(L2K_SOURCE_VISION_3D)
    public let maxTracked = 1
    public let providesImages = true
    public var onFrame: ((L2KSourceFrame) -> Void)?

    private let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "l2k.source.ultrawide")
    private let motion = CMMotionManager()
    private let detector = VisionHybridDetector()
    private let masker = VisionPersonMask()
    private var device: AVCaptureDevice?
    private var lastMask: CVPixelBuffer?
    private var bodyVisible = false
    private var trackingID: UInt32 = 0
    private var seenRate = JointSeenRate()

    private let stateLock = NSLock()
    private var lastStatus = "Starting"
    private var lastDiagnostics: [String] = []
    private var frames = 0
    private var rateStart: TimeInterval = 0, rateCount = 0, rate = 0.0

    /// The longest exposure allowed is 1/this second.
    public static let shortestExposureDenominator: Int32 = 120

    public static var camera: AVCaptureDevice? {
        AVCaptureDevice.default(.builtInUltraWideCamera, for: .video, position: .back)
    }

    public static var isSupported: Bool { camera != nil }

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
        guard let camera = Self.camera else {
            throw L2KSourceError.unsupported("This device has no rear ultra-wide camera")
        }
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .denied, .restricted:
            throw L2KSourceError.unsupported("Camera access is off for Lidar2Kinect (Settings > Privacy > Camera)")
        case .notDetermined:
            setStatus("Waiting for camera permission", diagnostics: [])
            AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                if granted { self?.configureAndRun(camera) } else { self?.setStatus("Camera access was refused", diagnostics: []) }
            }
        default:
            configureAndRun(camera)
        }
        if motion.isDeviceMotionAvailable {
            motion.deviceMotionUpdateInterval = 1.0 / 60
            motion.startDeviceMotionUpdates()
        }
    }

    public func stop() {
        motion.stopDeviceMotionUpdates()
        queue.async { [session] in
            if session.isRunning { session.stopRunning() }
        }
    }

    private func configureAndRun(_ camera: AVCaptureDevice) {
        queue.async {
            self.device = camera
            self.session.beginConfiguration()
            self.session.inputs.forEach { self.session.removeInput($0) }
            self.session.outputs.forEach { self.session.removeOutput($0) }
            self.session.sessionPreset = .hd1280x720
            guard let input = try? AVCaptureDeviceInput(device: camera), self.session.canAddInput(input) else {
                self.session.commitConfiguration()
                self.setStatus("The ultra-wide camera could not be opened", diagnostics: [])
                return
            }
            self.session.addInput(input)
            let output = AVCaptureVideoDataOutput()
            output.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
            output.alwaysDiscardsLateVideoFrames = true // if Vision falls behind, frames are dropped, not queued
            output.setSampleBufferDelegate(self, queue: self.queue)
            guard self.session.canAddOutput(output) else {
                self.session.commitConfiguration()
                self.setStatus("The camera output could not be set up", diagnostics: [])
                return
            }
            self.session.addOutput(output)
            if let connection = output.connection(with: .video), connection.isCameraIntrinsicMatrixDeliverySupported {
                connection.isCameraIntrinsicMatrixDeliveryEnabled = true
            }
            self.session.commitConfiguration()
            if (try? camera.lockForConfiguration()) != nil {
                // a Kinect delivers 30 frames per second
                camera.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)
                camera.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 30)
                // Short exposures: a fast-moving hand smeared across the frame is a hand the detector loses.
                // The camera raises its gain instead; a noisier picture costs the detector less than blur.
                camera.activeMaxExposureDuration = CMTime(value: 1, timescale: Self.shortestExposureDenominator)
                camera.unlockForConfiguration()
            }
            self.session.startRunning()
        }
    }

    // MARK: - frames (on `queue`)

    public func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let image = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        let now = ProcessInfo.processInfo.systemUptime
        frames += 1
        rateCount += 1
        if rateStart == 0 { rateStart = now }
        if now - rateStart >= 1 {
            rate = Double(rateCount) / (now - rateStart)
            rateStart = now
            rateCount = 0
        }
        let width = Float(CVPixelBufferGetWidth(image)), height = Float(CVPixelBufferGetHeight(image))

        // The camera's pinhole model: delivered with the frame, else from the format's field of view.
        var camera: l2k_src_camera
        var intrinsicsFromFrame = false
        if let data = CMGetAttachment(sampleBuffer, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, attachmentModeOut: nil) as? Data,
           data.count >= MemoryLayout<matrix_float3x3>.size {
            let k = data.withUnsafeBytes { $0.loadUnaligned(as: matrix_float3x3.self) }
            camera = l2k_src_camera(fxn: k.columns.0.x / width, fyn: k.columns.1.y / height, cxn: k.columns.2.x / width, cyn: k.columns.2.y / height)
            intrinsicsFromFrame = true
        } else {
            let fov = (device?.activeFormat.videoFieldOfView ?? 100) * .pi / 180 // horizontal, degrees
            let fx = (width / 2) / tan(fov / 2)
            camera = l2k_src_camera(fxn: fx / width, fyn: fx / height, cxn: 0.5, cyn: 0.5)
        }

        // World up in camera space. Core Motion's device axes: x to the right of the screen and y to its top in
        // portrait, z out of the screen. The rear camera's image has x along -y(device) and y along +x(device).
        var up = SIMD3<Float>(0, 1, 0)
        var haveGravity = false
        if let g = motion.deviceMotion?.gravity {
            up = simd_normalize(SIMD3(Float(g.y), Float(-g.x), Float(-g.z)))
            haveGravity = true
        }
        let quadrant = l2k_quadrant_from_up(up.x, up.y)

        let visionStart = ProcessInfo.processInfo.systemUptime
        let joints = detector.detect(in: image, quadrant: quadrant, camera: camera)
        let poseMs = (ProcessInfo.processInfo.systemUptime - visionStart) * 1000
        // The silhouette changes slowly compared with the pose: refresh the mask on every second frame.
        if frames % 2 == 1 || lastMask == nil { lastMask = masker.mask(of: image) }
        let visionMs = (ProcessInfo.processInfo.systemUptime - visionStart) * 1000

        var bodies: [l2k_src_body] = []
        if !joints.isEmpty {
            seenRate.note(detector.lastSeenSources)
            if !bodyVisible {
                bodyVisible = true
                trackingID = trackingID == UInt32.max ? 1 : trackingID + 1 // a body that reappears is a new skeleton
            }
            var body = l2k_src_body()
            body.tracking_id = trackingID
            VisionBody3DDetector.place(joints, camera: camera, into: &body)
            bodies.append(body)
        } else {
            bodyVisible = false
        }

        if frames % 8 == 1 {
            var lines = [String(format: "Camera frames %d (%.0f/s), image %d×%d, roll quadrant %d", frames, rate, Int(width), Int(height), quadrant)]
            lines.append(String(format: "Field of view %.0f° × %.0f° (%@)", 2 * atan(0.5 / camera.fxn) * 180 / .pi, 2 * atan(0.5 / camera.fyn) * 180 / .pi,
                                intrinsicsFromFrame ? "from the camera" : "from the format"))
            lines.append("Gravity: \(haveGravity ? "from the motion sensors" : "not available, assuming the device is upright")"
                         + String(format: ", up in camera space (%.2f %.2f %.2f)", up.x, up.y, up.z))
            if !joints.isEmpty {
                // where Vision put each joint in the camera buffer (u right, v down), and how far away
                lines.append("Joints u,v,m: " + joints.map { String(format: "%d:%.2f,%.2f,%.1f", $0.source, $0.u, $0.v, $0.depth) }.joined(separator: " "))
            }
            lines.append(String(format: "Vision 2D + 3D pose: %d joints (%d from the picture, %d from the model), %.0f ms; with mask %.0f ms",
                                joints.count, detector.lastSeen, detector.lastModelOnly, poseMs, visionMs)
                         + (detector.lastError.map { " — \($0)" } ?? ""))
            lines.append(seenRate.summary)
            if !joints.isEmpty {
                lines.append(String(format: "Body height %.2f m (%@)", detector.lastHeight, detector.lastHeightMeasured ? "measured" : "assumed"))
            }
            lines.append("Person mask (Vision): \(lastMask == nil ? "none" : "present")" + (masker.lastError.map { " — \($0)" } ?? ""))
            setStatus(joints.isEmpty ? "No body in view · ultra-wide camera, no LiDAR" : "Body seen · ultra-wide camera, no LiDAR", diagnostics: lines)
        }

        onFrame?(L2KSourceFrame(timestamp: now, bodies: bodies, upCamera: up, camera: camera,
                                depthMap: nil, segmentation: lastMask, image: image))
    }

    private func setStatus(_ status: String, diagnostics: [String]) {
        stateLock.lock()
        lastStatus = status
        lastDiagnostics = diagnostics
        stateLock.unlock()
    }
}
#endif
