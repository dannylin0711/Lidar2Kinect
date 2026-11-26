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
// Latency: the skeleton leaves this source as soon as Vision has it; the person mask (for the depth image) is made
// afterwards and only when an image is wanted. `captureRate` 60 runs the camera at 60 frames a second where a format
// allows it and still detects on 30 of them: a picture is then younger when it arrives (measured: "Timing" line).
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

    /// - Parameter captureRate: frames per second asked of the camera, 30 or 60. 60: the format with the widest view
    ///   among those that run that fast (it can show less than the 30 fps one; 30 again when the camera has none).
    ///   The body is still detected 30 times a second, on the newest frame, unless a receiver asks for more
    ///   (`requestFrameRate`): then on every frame Vision keeps up with.
    public init(lens: Lens = .rearUltraWide, captureRate: Double = 30) {
        self.lens = lens
        self.captureRate = captureRate
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
    private var maskFrame = 0, maskMs = 0.0, masksMade = 0
    private let captureRate: Double
    private var runningRate = 30.0
    /// The body is detected on at most this many camera frames a second (`requestFrameRate`; on `queue`).
    private var detectionRate = 30.0
    private var lastUsedTime: Double?
    private var used = 0, usedCount = 0, usedRate = 0.0
    private var readyMs = 0.0
    private var bodyVisible = false
    private var trackingID: UInt32 = 0
    private var seenRate = JointSeenRate()
    private var visionQuadrant: Int32 = -1
    private var workMs = 0.0, delayMs = 0.0

    /// For troubleshooting: use the capture format of this size ("1280x720") instead of the one with the widest view.
    public static var formatOverride: String?
    /// Give Vision the camera's intrinsics, so that its distances are true ones (see VisionCanvas.swift).
    /// The limbs' depths as before 0.9.4: towards or away from the camera as Vision's 3D model has it, with the model's
    /// bone lengths. Off (default): the 0.9.4 rule (`VisionHybridDetector.DepthRule.adaptive`, lengths from the picture).
    public var depthFromModel: Bool {
        get { detector.depthRule == .model }
        set {
            detector.depthRule = newValue ? .model : .adaptive
            detector.lengthSource = newValue ? .model : .picture
        }
    }

    /// The camera's rate as it runs (30 until it has started; 60 only with `captureRate` 60 and a format that allows it).
    public var maxFrameRate: Double {
        stateLock.lock()
        defer { stateLock.unlock() }
        return publishedRate
    }
    private var publishedRate = 30.0

    public func requestFrameRate(_ rate: Double) { detectionRate = max(1, rate) }

    public var usesCameraIntrinsics: Bool {
        get { detector.givesIntrinsics }
        set { detector.givesIntrinsics = newValue }
    }

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

    /// Whether the lens's camera can run at 60 frames a second and still show what it shows at 30 (then the faster
    /// camera only costs power). iPad Pro 11" (M2) front camera: yes (1920×1440, 107° × 91° at both rates); iPhone 17
    /// Pro front camera: no (84° × 84° at 30, 84° × 68° at 60).
    public static func fastCameraKeepsView(_ lens: Lens) -> Bool {
        guard let device = camera(for: lens) else { return false }
        return L2KCameraView.fastFormatKeepsView(formats(of: device).map(\.info))
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
    public static func formatReport(for lens: Lens, captureRate: Double = 30) -> [String] {
        guard let device = camera(for: lens) else { return ["\(lens.rawValue): no such camera"] }
        let all = formats(of: device)
        let chosen = L2KCameraView.widestFormat(all.map(\.info), frameRate: captureRate)
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
            self.lastUsedTime = nil
            self.runningRate = 30
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
            var chosen = L2KCameraView.widestFormat(all.map(\.info), frameRate: self.captureRate)
            var forced = false
            if let wanted = Self.formatOverride?.lowercased().split(separator: "x").compactMap({ Int($0) }), wanted.count == 2 {
                let same = all.indices.filter { all[$0].info.width == wanted[0] && all[$0].info.height == wanted[1] && all[$0].info.maxFrameRate >= 29.5 }
                if let index = same.max(by: { all[$0].info.viewArea < all[$1].info.viewArea }) {
                    chosen = index
                    forced = true
                }
            }
            let locked = (try? camera.lockForConfiguration()) != nil
            if locked {
                if let chosen { camera.activeFormat = all[chosen].format }
                // The ultra-wide lens bends straight lines; the correction gives the pinhole picture the maths assumes.
                if camera.isGeometricDistortionCorrectionSupported { camera.isGeometricDistortionCorrectionEnabled = true }
                let format = camera.activeFormat
                // A Kinect delivers 30 frames per second. A faster camera (captureRate 60) still gives 30 skeletons a
                // second, each from a picture that took less time to get here.
                func runs(_ rate: Double) -> Bool {
                    format.videoSupportedFrameRateRanges.contains { $0.minFrameRate <= rate && $0.maxFrameRate >= rate - 0.5 }
                }
                let rate = self.captureRate > 30 && runs(self.captureRate) ? self.captureRate : 30
                if runs(rate) {
                    camera.activeVideoMinFrameDuration = CMTime(value: 1, timescale: Int32(rate.rounded()))
                    camera.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: Int32(rate.rounded()))
                    self.runningRate = rate
                }
                self.stateLock.lock()
                self.publishedRate = self.runningRate
                self.stateLock.unlock()
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
                var how = forced ? "asked for (-l2kFormat)" : "the widest view"
                if !forced, self.captureRate > 30 {
                    how = self.runningRate > 30 ? String(format: "the widest view at %.0f frames a second", self.runningRate)
                                                : String(format: "the widest view (no format runs at %.0f frames a second)", self.captureRate)
                }
                self.formatLine = String(format: "Format %d×%d at %.0f fps, %@ of %d formats: %.0f° × %.0f°", info.width, info.height,
                                         self.runningRate, how, all.count, info.fieldOfView, info.verticalFieldOfView)
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
        let taken = CMSampleBufferGetPresentationTimeStamp(sampleBuffer).seconds
        frames += 1
        rateCount += 1
        if rateStart == 0 { rateStart = now }
        if now - rateStart >= 1 {
            rate = Double(rateCount) / (now - rateStart)
            usedRate = Double(usedCount) / (now - rateStart)
            rateStart = now
            rateCount = 0
            usedCount = 0
        }
        // A camera running faster than the detection rate (60 against 30): the body is detected on every second frame.
        if taken.isFinite {
            guard L2KCameraView.usesFrame(at: taken, last: lastUsedTime, rate: detectionRate) else { return }
            lastUsedTime = taken
        }
        used += 1
        usedCount += 1
        // How old the picture already is when it arrives here (exposure to delivery).
        let age = CMTimeSubtract(CMClockGetTime(CMClockGetHostTimeClock()), CMSampleBufferGetPresentationTimeStamp(sampleBuffer)).seconds
        if age.isFinite { delayMs = delayMs == 0 ? age * 1000 : delayMs * 0.9 + age * 100 }
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
        // The system's level angle is the turn the connection has to apply to the camera's own frames. The frames
        // here have already been turned by the connection's angle - not 0 for every camera: the iPhone 17 Pro's front
        // camera has a level angle of 0° in portrait, yet its frames arrive lying on their side as an older front
        // camera's do (seen 2026-11-11) - so what is left to turn is the difference.
        let connectionAngle = Float(connection.videoRotationAngle)
        let levelAngle = rotation.map { L2KCameraView.remainingTurn(levelAngle: Float($0.videoRotationAngleForHorizonLevelCapture), applied: connectionAngle) }
        var up = SIMD3<Float>(0, 1, 0)
        var haveGravity = false
        if let g = motion.deviceMotion?.gravity {
            let assumed = lens == .front ? SIMD3(Float(g.y), Float(g.x), Float(g.z)) : SIMD3(Float(g.y), Float(-g.x), Float(-g.z))
            if simd_length(assumed) > 1e-3 {
                up = mounting.correct(up: simd_normalize(assumed), levelAngle: levelAngle)
                haveGravity = true
            }
        }
        // Which way is up for Vision. A device lying flat has no "up" in its picture: the roll then stays as it was,
        // and it only changes when the picture is clearly turned (the picture for Vision changes shape with it).
        let candidate = l2k_quadrant_from_up(up.x, up.y)
        if visionQuadrant < 0 {
            visionQuadrant = candidate
        } else if candidate != visionQuadrant, simd_length(SIMD2(up.x, up.y)) > 0.35, abs(abs(up.x) - abs(up.y)) > 0.25 {
            visionQuadrant = candidate
        }
        let quadrant = visionQuadrant

        let visionStart = ProcessInfo.processInfo.systemUptime
        let joints = detector.detect(in: image, quadrant: quadrant, camera: camera)
        let poseMs = (ProcessInfo.processInfo.systemUptime - visionStart) * 1000
        // The person mask is made when an image asks for it, which is after the skeleton has been handed on (the
        // pipeline makes the skeleton first). The silhouette changes slowly compared with the pose: a mask is kept
        // for two frames. Runs on this queue, inside `onFrame` below.
        let index = used
        let makeMask: () -> CVPixelBuffer? = { [self] in
            if lastMask == nil || index - maskFrame >= 2 {
                let start = ProcessInfo.processInfo.systemUptime
                lastMask = masker.mask(of: image)
                maskFrame = index
                masksMade += 1
                let ms = (ProcessInfo.processInfo.systemUptime - start) * 1000
                maskMs = maskMs == 0 ? ms : maskMs * 0.8 + ms * 0.2
            }
            return lastMask
        }

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

        if used % 8 == 1 {
            var lines = [String(format: "Camera frames %d (%.0f/s, %.0f/s used), image %d×%d, roll quadrant %d", frames, rate, usedRate, Int(width), Int(height), quadrant)]
            lines.append(String(format: "Field of view %.0f° × %.0f° (%@)", 2 * atan(0.5 / camera.fxn) * 180 / .pi, 2 * atan(0.5 / camera.fyn) * 180 / .pi,
                                intrinsicsFromFrame ? "from the camera" : "from the format"))
            if !formatLine.isEmpty { lines.append(formatLine) }
            lines.append("Gravity: \(haveGravity ? "from the motion sensors" : "not available, assuming the device is upright")"
                         + String(format: ", up in camera space (%.2f %.2f %.2f)", up.x, up.y, up.z))
            lines.append(mounting.summary + (rotation.map { String(format: " (level angle %.0f°, frames already turned %.0f°)", $0.videoRotationAngleForHorizonLevelCapture, connectionAngle) } ?? ""))
            if !joints.isEmpty {
                // where Vision put each joint in the camera buffer (u right, v down), and how far away
                lines.append("Joints u,v,m: " + joints.map { String(format: "%d:%.2f,%.2f,%.1f", $0.source, $0.u, $0.v, $0.depth) }.joined(separator: " "))
            }
            lines.append(String(format: "Vision 2D + 3D pose: %d joints (%d from the picture, %d from the model), %.0f ms",
                                joints.count, detector.lastSeen, detector.lastModelOnly, poseMs)
                         + (detector.lastError.map { " — \($0)" } ?? ""))
            if let layout = detector.lastLayout {
                var line = String(format: "Picture for Vision %d×%d (camera image %.0f×%.0f in it), drawn in %.1f ms", layout.width, layout.height,
                                  layout.content.width, layout.content.height, detector.lastDrawMs)
                if !usesCameraIntrinsics {
                    line += "; Vision's own camera (-l2kVisionCamera 0)"
                } else if let used = detector.lastFocalUsed {
                    line += String(format: "; focal length given %.0f px, used by Vision %.0f px", detector.lastFocalGiven, used)
                } else {
                    line += String(format: "; focal length given %.0f px", detector.lastFocalGiven)
                }
                lines.append(line)
            }
            lines.append(String(format: "Timing: picture %.0f ms old on arrival, skeleton handed on %.0f ms after the picture was taken, %.0f ms of work per frame, device %@",
                                delayMs, readyMs, workMs, Self.thermalName))
            lines.append(seenRate.summary)
            if !joints.isEmpty {
                lines.append(String(format: "Body height %.2f m (%@)", detector.lastHeight, detector.lastHeightMeasured ? "measured" : "assumed"))
            }
            lines.append((masksMade == 0 ? "Person mask (Vision): not asked for"
                                         : String(format: "Person mask (Vision): %@, made after the skeleton in %.0f ms", lastMask == nil ? "none" : "present", maskMs))
                         + (masker.lastError.map { " — \($0)" } ?? ""))
            setStatus((joints.isEmpty ? "No body in view" : "Body seen") + " · \(lensName), no LiDAR", diagnostics: lines)
        }

        var frame = L2KSourceFrame(timestamp: now, bodies: bodies, upCamera: up, camera: camera, depthMap: nil, segmentation: nil, image: image)
        frame.makeSegmentation = makeMask
        // The picture's age when the skeleton leaves here (the pipeline hands it on within a millisecond).
        if age.isFinite {
            let ready = (age + ProcessInfo.processInfo.systemUptime - now) * 1000
            readyMs = readyMs == 0 ? ready : readyMs * 0.9 + ready * 0.1
        }
        onFrame?(frame)
        let work = (ProcessInfo.processInfo.systemUptime - now) * 1000
        workMs = workMs == 0 ? work : workMs * 0.9 + work * 0.1
    }

    private static var thermalName: String {
        switch ProcessInfo.processInfo.thermalState {
        case .nominal: return "cool"
        case .fair: return "warm"
        case .serious: return "hot (the system slows it down)"
        case .critical: return "very hot (the system slows it down hard)"
        @unknown default: return "?"
        }
    }

    private func setStatus(_ status: String, diagnostics: [String]) {
        stateLock.lock()
        lastStatus = status
        lastDiagnostics = diagnostics
        stateLock.unlock()
    }
}
#endif
