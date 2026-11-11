// Capture source: one camera alone, with Vision's body pose. No LiDAR. Two lenses: the rear ultra-wide camera, and
// the front camera (the one that sees the player when the game is on this device's own screen).
//
// The LiDAR only covers the normal wide camera's view (and ARKit offers only that camera - read from its
// supported video formats on an iPhone 17 Pro), so a wider view means doing without measured depth. The
// ultra-wide camera sees a whole standing person from about a metre away. Vision estimates the 3D pose of
// one person from the image; the depth image is the person mask at the body's distance, or the figure drawn
// from the skeleton.
//
// The whole view is used: the capture format is the one that shows the most (a 4:3 format reads the whole sensor;
// the 16:9 ones cut the top and the bottom off), nothing that crops is on (stabilisation, Center Stage, zoom), and
// Vision is given the whole frame.
//
// Distances are Vision's estimate (it assumes a body height when it cannot measure one), so the skeleton is
// right in shape and direction but only roughly in scale. Dance Evolution judges limb directions, not size.
#if os(iOS)
import AVFoundation
import CoreMotion
import Foundation
import L2KShared

public final class UltraWideSource: NSObject, L2KSource, AVCaptureVideoDataOutputSampleBufferDelegate {
    /// Which camera the source runs on.
    public enum Lens: String {
        /// the rear ultra-wide camera
        case rearUltraWide
        /// the camera on the screen's side (the widest one the device has there)
        case front
    }

    public let kind = UInt8(L2K_SOURCE_VISION_3D)
    public let maxTracked = 1
    public let providesImages = true
    public var onFrame: ((L2KSourceFrame) -> Void)?
    public let lens: Lens

    public init(lens: Lens = .rearUltraWide) {
        self.lens = lens
        // The rear camera's mounting is known (it ran on a device); the front camera's differs between devices.
        mounting = L2KCameraMounting(adopts: lens == .front)
        super.init()
    }

    private let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "l2k.source.ultrawide")
    private let motion = CMMotionManager()
    private let detector = VisionHybridDetector()
    private let masker = VisionPersonMask()
    private var device: AVCaptureDevice?
    private var rotation: AVCaptureDevice.RotationCoordinator?
    private var mounting: L2KCameraMounting
    private var formatLine = ""
    private var formatFieldOfView: Float = 0
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

    public static var camera: AVCaptureDevice? { camera(for: .rearUltraWide) }

    public static var isSupported: Bool { camera != nil }

    public static func isSupported(_ lens: Lens) -> Bool { camera(for: lens) != nil }

    /// The device for a lens. Front: of the cameras on the screen's side, the one that can show the most.
    public static func camera(for lens: Lens) -> AVCaptureDevice? {
        switch lens {
        case .rearUltraWide:
            return AVCaptureDevice.default(.builtInUltraWideCamera, for: .video, position: .back)
        case .front:
            let found = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInUltraWideCamera, .builtInWideAngleCamera, .builtInTrueDepthCamera],
                                                         mediaType: .video, position: .front).devices
            func view(_ device: AVCaptureDevice) -> Float {
                let all = formats(of: device).map(\.info)
                return L2KCameraView.widestFormat(all).map { all[$0].tanHalfWidth * all[$0].tanHalfHeight } ?? 0
            }
            return found.max { view($0) < view($1) }
        }
    }

    /// The device's 8-bit video formats with what matters for choosing among them.
    static func formats(of device: AVCaptureDevice) -> [(format: AVCaptureDevice.Format, info: L2KCameraFormat)] {
        let corrected = device.isGeometricDistortionCorrectionSupported
        func describe(_ format: AVCaptureDevice.Format) -> L2KCameraFormat {
            let size = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let correctedView = format.geometricDistortionCorrectedVideoFieldOfView
            let view = corrected && correctedView > 0 ? correctedView : format.videoFieldOfView
            let rate = format.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0
            return L2KCameraFormat(width: Int(size.width), height: Int(size.height), fieldOfView: view, maxFrameRate: rate)
        }
        let plain = device.formats.filter {
            let type = CMFormatDescriptionGetMediaSubType($0.formatDescription)
            return type == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange || type == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        }
        return (plain.isEmpty ? device.formats : plain).map { ($0, describe($0)) }
    }

    /// For troubleshooting: every format of the lens's camera and the one the source would pick.
    public static func formatReport(for lens: Lens) -> [String] {
        guard let device = camera(for: lens) else { return ["\(lens.rawValue): no such camera"] }
        let all = formats(of: device)
        let chosen = L2KCameraView.widestFormat(all.map(\.info))
        var lines = ["\(lens.rawValue): \(device.localizedName) (\(device.deviceType.rawValue)), \(all.count) formats"]
        var seen = Set<String>()
        for (index, entry) in all.enumerated() {
            let line = String(format: "%d×%d view %.1f° × %.1f° up to %.0f fps", entry.info.width, entry.info.height,
                              entry.info.fieldOfView, entry.info.verticalFieldOfView, entry.info.maxFrameRate)
            if index == chosen { lines.append("  chosen: " + line) } else if seen.insert(line).inserted { lines.append("  " + line) }
        }
        return lines
    }

    private var lensName: String { lens == .front ? "front camera" : "ultra-wide camera" }

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
        guard let camera = Self.camera(for: lens) else {
            throw L2KSourceError.unsupported(lens == .front ? "This device has no front camera" : "This device has no rear ultra-wide camera")
        }
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .denied, .restricted:
            throw L2KSourceError.unsupported("Camera access is off for this app (Settings > Privacy > Camera)")
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
            self.rotation = AVCaptureDevice.RotationCoordinator(device: camera, previewLayer: nil)
            if self.lens == .front {
                // Center Stage pans and crops the front camera's picture to follow a person: the view must stay whole
                // and the picture must stay where the camera's intrinsics say it is.
                AVCaptureDevice.centerStageControlMode = .app
                AVCaptureDevice.isCenterStageEnabled = false
            }
            self.session.beginConfiguration()
            self.session.inputs.forEach { self.session.removeInput($0) }
            self.session.outputs.forEach { self.session.removeOutput($0) }
            self.session.sessionPreset = .inputPriority // the format is chosen below, not by a preset
            guard let input = try? AVCaptureDeviceInput(device: camera), self.session.canAddInput(input) else {
                self.session.commitConfiguration()
                self.setStatus("The \(self.lensName) could not be opened", diagnostics: [])
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

            // The format that shows the most. The device stays locked until the session runs, so the session does not
            // put its own choice back.
            let all = Self.formats(of: camera)
            let chosen = L2KCameraView.widestFormat(all.map(\.info))
            let locked = (try? camera.lockForConfiguration()) != nil
            if locked {
                if let chosen { camera.activeFormat = all[chosen].format }
                // The ultra-wide lens bends straight lines; the correction gives the pinhole picture the maths assumes.
                if camera.isGeometricDistortionCorrectionSupported { camera.isGeometricDistortionCorrectionEnabled = true }
                let format = camera.activeFormat
                // a Kinect delivers 30 frames per second
                if format.videoSupportedFrameRateRanges.contains(where: { $0.minFrameRate <= 30 && $0.maxFrameRate >= 30 }) {
                    camera.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)
                    camera.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 30)
                }
                // Short exposures: a fast-moving hand smeared across the frame is a hand the detector loses.
                // The camera raises its gain instead; a noisier picture costs the detector less than blur.
                let shortest = CMTime(value: 1, timescale: Self.shortestExposureDenominator)
                if CMTimeCompare(shortest, format.minExposureDuration) >= 0, CMTimeCompare(shortest, format.maxExposureDuration) <= 0 {
                    camera.activeMaxExposureDuration = shortest
                }
                camera.videoZoomFactor = camera.minAvailableVideoZoomFactor // zoomed all the way out
            }
            if let chosen {
                let info = all[chosen].info
                self.formatFieldOfView = info.fieldOfView
                self.formatLine = String(format: "Format %d×%d, the widest view of %d formats: %.0f° × %.0f°", info.width, info.height,
                                         all.count, info.fieldOfView, info.verticalFieldOfView)
            }
            if let connection = output.connection(with: .video) {
                // The picture as the camera sees it: not mirrored (a front camera's can be), not stabilised (that crops).
                connection.automaticallyAdjustsVideoMirroring = false
                if connection.isVideoMirroringSupported { connection.isVideoMirrored = false }
                if connection.isVideoStabilizationSupported { connection.preferredVideoStabilizationMode = .off }
                if connection.isCameraIntrinsicMatrixDeliverySupported { connection.isCameraIntrinsicMatrixDeliveryEnabled = true }
            }
            self.session.commitConfiguration()
            self.session.startRunning()
            if locked { camera.unlockForConfiguration() }
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
            let fov = (formatFieldOfView > 0 ? formatFieldOfView : device?.activeFormat.videoFieldOfView ?? 100) * .pi / 180 // horizontal, degrees
            let fx = (width / 2) / tan(fov / 2)
            camera = l2k_src_camera(fxn: fx / width, fyn: fx / height, cxn: 0.5, cyn: 0.5)
        }

        // World up in camera space (x image right, y image up, z towards the viewer behind the camera). Core Motion's
        // device axes: x to the right of the screen and y to its top in portrait, z out of the screen. The rear
        // camera's image has x along -y(device) and y along +x(device), and it looks along -z(device). The front
        // camera looks along +z(device); its image, not mirrored, is assumed to have x along -y(device) and y along
        // -x(device) (a sensor upright with the device held the other way round in landscape), and `mounting` puts
        // that right from the system's own level angle if this device's front camera sits differently.
        var up = SIMD3<Float>(0, 1, 0)
        var haveGravity = false
        if let g = motion.deviceMotion?.gravity {
            let assumed = lens == .front ? SIMD3(Float(g.y), Float(g.x), Float(g.z)) : SIMD3(Float(g.y), Float(-g.x), Float(-g.z))
            if simd_length(assumed) > 1e-3 {
                up = mounting.correct(up: simd_normalize(assumed), levelAngle: rotation.map { Float($0.videoRotationAngleForHorizonLevelCapture) })
                haveGravity = true
            }
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
            if !formatLine.isEmpty { lines.append(formatLine) }
            lines.append("Gravity: \(haveGravity ? "from the motion sensors" : "not available, assuming the device is upright")"
                         + String(format: ", up in camera space (%.2f %.2f %.2f)", up.x, up.y, up.z))
            lines.append(mounting.summary + (rotation.map { String(format: " (level angle %.0f°)", $0.videoRotationAngleForHorizonLevelCapture) } ?? ""))
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
            setStatus((joints.isEmpty ? "No body in view" : "Body seen") + " · \(lensName), no LiDAR", diagnostics: lines)
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
