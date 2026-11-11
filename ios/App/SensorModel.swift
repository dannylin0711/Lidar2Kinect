// App state: owns the capture session, the sender and the receiver discovery, and feeds the UI.
import CoreImage
import ImageIO
import L2KKit
import L2KShared
import SwiftUI

@MainActor
final class SensorModel: ObservableObject {
    enum SourceChoice: String, CaseIterable, Identifiable {
        case lidar, ultrawide, front, arkit, synthetic
        var id: String { rawValue }
        var title: String {
            switch self {
            case .lidar: return "LiDAR depth"
            case .ultrawide: return "Ultra-wide"
            case .front: return "Front camera"
            case .arkit: return "Body tracking"
            case .synthetic: return "Test figure"
            }
        }
    }

    enum PreviewMode: String, CaseIterable, Identifiable {
        case color, depth
        var id: String { rawValue }
        var title: String { self == .color ? "Colour" : "Depth" }
    }

    @Published var sourceChoice: SourceChoice
    @Published var previewMode: PreviewMode = .color { didSet { applyPreviewMode() } }
    @Published var upperBodyOnly: Bool {
        didSet {
            UserDefaults.standard.set(upperBodyOnly, forKey: "l2kUpperBody")
            session.pipeline.upperBodyMode = upperBodyOnly
        }
    }
    @Published var sendColor: Bool {
        didSet {
            UserDefaults.standard.set(sendColor, forKey: "l2kSendColor")
            session.colorEnabled = sendColor
        }
    }
    @Published var fitPlayArea: Bool {
        didSet {
            UserDefaults.standard.set(fitPlayArea, forKey: "l2kFitArea")
            session.pipeline.fitPlayArea = fitPlayArea
        }
    }
    /// Where the player really stands (metres from the sensor), before the play-area fit.
    @Published var realDistance: Float?
    /// The skeletons as the camera sees them (before the fit / the normalising), for drawing over the picture.
    @Published var pictureSkeleton: l2k_skeleton_frame?
    @Published var normalize: Bool {
        didSet {
            UserDefaults.standard.set(normalize, forKey: "l2kNormalize")
            session.pipeline.normalize = normalize
        }
    }
    @Published var normalizePlayers: Int {
        didSet {
            UserDefaults.standard.set(normalizePlayers, forKey: "l2kNormalizePlayers")
            session.pipeline.normalizePlayers = normalizePlayers
        }
    }
    @Published var keepPlayer: Bool {
        didSet {
            UserDefaults.standard.set(keepPlayer, forKey: "l2kKeepPlayer")
            session.pipeline.keepIdentity = keepPlayer
        }
    }
    @Published var sourceStatus = "Not started"
    @Published var sourceError: String?
    @Published var senderStatus = L2KSender.Status()
    @Published var receivers: [L2KDiscovery.Receiver] = []
    @Published var preview: CGImage?
    @Published var skeleton: l2k_skeleton_frame?
    @Published var frameRate: Double = 0
    /// The colour preview shows the camera's whole view, not only the part the Kinect's image covers.
    private var showsWholeCamera = false
    /// The zoom of the picture currently shown (the depth preview is never zoomed).
    @Published var shownZoom: CGFloat = 1
    @Published var diagnostics: [String] = []
    @Published var recording: L2KRecorder.State = .idle
    @Published var host: String { didSet { UserDefaults.standard.set(host, forKey: "l2kHost") } }
    @Published var port: String { didSet { UserDefaults.standard.set(port, forKey: "l2kPort") } }

    /// The sources this device can run (the test figure always; the camera sources never in the simulator).
    let availableSources: [SourceChoice]
    /// What ARKit supports on this device, for the diagnostics card.
    let deviceSupport: [(label: String, supported: Bool)]
    var cameraAvailable: Bool { availableSources.count > 1 }
    private let session = L2KSession()
    private let discovery = L2KDiscovery()
    private let previewer = Previewer()
    private var started = false
    private let players: Int

    init() {
        let defaults = UserDefaults.standard
        var sources: [SourceChoice] = []
        #if targetEnvironment(simulator)
        deviceSupport = []
        #else
        if VisionDepthSource.isSupported { sources.append(.lidar) }
        if UltraWideSource.isSupported { sources.append(.ultrawide) }
        if UltraWideSource.isSupported(.front) { sources.append(.front) }
        if ARKitBodySource.isSupported { sources.append(.arkit) }
        deviceSupport = L2KDeviceSupport.report()
        #endif
        sources.append(.synthetic)
        availableSources = sources
        // Launch arguments (also plain defaults): -l2kSource lidar|ultrawide|front|arkit|synthetic, -l2kHost, -l2kPort, -l2kPlayers, -l2kAutoConnect 1
        // Default: the first camera source the device supports (LiDAR depth before 3D body tracking).
        let wanted = SourceChoice(rawValue: defaults.string(forKey: "l2kSource") ?? "")
        sourceChoice = wanted.flatMap { sources.contains($0) ? $0 : nil } ?? sources[0]
        host = defaults.string(forKey: "l2kHost") ?? ""
        port = defaults.string(forKey: "l2kPort") ?? String(L2K.defaultPort)
        players = max(1, defaults.integer(forKey: "l2kPlayers"))
        previewMode = PreviewMode(rawValue: defaults.string(forKey: "l2kPreview") ?? "") ?? .color // -l2kPreview depth
        upperBodyOnly = defaults.bool(forKey: "l2kUpperBody") // also a launch argument: -l2kUpperBody 1
        keepPlayer = defaults.object(forKey: "l2kKeepPlayer") as? Bool ?? true // -l2kKeepPlayer 0
        sendColor = defaults.object(forKey: "l2kSendColor") as? Bool ?? true // -l2kSendColor 0
        normalize = defaults.bool(forKey: "l2kNormalize") // -l2kNormalize 1
        normalizePlayers = defaults.object(forKey: "l2kNormalizePlayers") as? Int ?? 1 // -l2kNormalizePlayers 2
        fitPlayArea = defaults.object(forKey: "l2kFitArea") as? Bool ?? true // -l2kFitArea 0
        session.pipeline.fitPlayArea = fitPlayArea
        session.pipeline.normalize = normalize
        session.pipeline.normalizePlayers = normalizePlayers
        session.colorEnabled = sendColor
        session.pipeline.upperBodyMode = upperBodyOnly
        session.pipeline.keepIdentity = keepPlayer
        session.deviceName = UIDevice.current.name
    }

    func startIfNeeded() {
        guard !started else { return }
        started = true
        session.sender.onStatus = { [weak self] status in
            DispatchQueue.main.async { self?.senderStatus = status }
        }
        discovery.onChange = { [weak self] receivers in self?.receivers = receivers }
        discovery.start()
        previewer.onPreview = { [weak self] image, zoom, skeleton, rate, status, diagnostics, distance, picture in
            guard let self else { return }
            self.realDistance = distance
            self.pictureSkeleton = picture
            self.preview = image
            self.shownZoom = zoom
            self.skeleton = skeleton
            self.frameRate = rate
            self.sourceStatus = status
            self.diagnostics = diagnostics
            self.logIfAsked(skeleton: skeleton, rate: rate)
        }
        applyPreviewMode()
        startSource()
        if UserDefaults.standard.bool(forKey: "l2kAutoConnect"), !host.isEmpty { connect() }
        let recordSeconds = UserDefaults.standard.double(forKey: "l2kRecord") // -l2kRecord 10: record after a 3 s delay
        if recordSeconds > 0 {
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in self?.record(seconds: recordSeconds) }
        }
        if UserDefaults.standard.bool(forKey: "l2kLog") {
            print("L2K-LOG sources: \(availableSources.map(\.rawValue).joined(separator: ",")) chosen: \(sourceChoice.rawValue)")
            for item in deviceSupport { print("L2K-LOG support \(item.supported ? "yes" : "no ") \(item.label)") }
            #if !targetEnvironment(simulator)
            for line in L2KDeviceSupport.videoFormats() { print("L2K-LOG " + line) }
            for lens in [UltraWideSource.Lens.rearUltraWide, .front] {
                for line in UltraWideSource.formatReport(for: lens) { print("L2K-LOG " + line) }
            }
            #endif
            fflush(stdout)
        }
    }

    /// -l2kLog 1: print the status and the diagnostics every two seconds (read with `devicectl ... launch --console`).
    private var lastLog: TimeInterval = 0
    private func logIfAsked(skeleton: l2k_skeleton_frame, rate: Double) {
        guard UserDefaults.standard.bool(forKey: "l2kLog") else { return }
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastLog >= 2 else { return }
        lastLog = now
        var lines = ["L2K-LOG source=\(sourceChoice.rawValue) upper=\(upperBodyOnly) out=\(String(format: "%.0f", rate))fps tracked=\(skeleton.trackedSkeletons.count) tilt=\(String(format: "%.0f", skeleton.elevationDegrees)) floor=\(String(format: "%.2f", skeleton.floor_plane.w)) | \(sourceStatus)"]
        if let body = skeleton.trackedSkeletons.first?.skeleton {
            let h = body.position(of: .head), l = body.position(of: .handLeft), r = body.position(of: .handRight), f = body.position(of: .footLeft)
            lines.append(String(format: "L2K-LOG   head (%.2f %.2f %.2f) handL (%.2f %.2f %.2f) handR (%.2f %.2f %.2f) footL (%.2f %.2f %.2f) states %@",
                                h.x, h.y, h.z, l.x, l.y, l.z, r.x, r.y, r.z, f.x, f.y, f.z, body.jointStates.map(String.init).joined()))
        }
        lines += diagnostics.map { "L2K-LOG   " + $0 }
        print(lines.joined(separator: "\n"))
        fflush(stdout)
    }

    /// Record the camera for a few seconds to Documents/l2k_record.mov (and .json), for troubleshooting on the Mac.
    func record(seconds: TimeInterval = 10) {
        guard let documents = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else { return }
        session.recorder.onState = { [weak self] state in self?.recording = state }
        session.recorder.start(url: documents.appendingPathComponent("l2k_record.mov"), seconds: seconds,
                               notes: ["source": sourceChoice.rawValue, "app": Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? ""])
    }

    func selectSource(_ choice: SourceChoice) {
        guard choice != sourceChoice else { return }
        sourceChoice = choice
        UserDefaults.standard.set(choice.rawValue, forKey: "l2kSource")
        startSource()
    }

    private func startSource() {
        let source: L2KSource
        #if targetEnvironment(simulator)
        source = SyntheticSource(players: players)
        #else
        switch sourceChoice {
        case .lidar:
            let lidar = VisionDepthSource()
            lidar.forceVisionMask = UserDefaults.standard.bool(forKey: "l2kVisionMask") // -l2kVisionMask 1
            source = lidar
        case .ultrawide: source = UltraWideSource()
        case .front: source = UltraWideSource(lens: .front)
        case .arkit: source = ARKitBodySource()
        case .synthetic: source = SyntheticSource(players: players)
        }
        #endif
        previewer.source = source
        previewer.context = session.pipeline.ciContext
        session.onFrame = { [previewer] frame in previewer.handle(frame) }
        // The ultra-wide and front cameras see far more than the Kinect's image: show everything they see.
        showsWholeCamera = sourceChoice == .ultrawide || sourceChoice == .front
        applyPreviewMode()
        do {
            try session.start(source)
            sourceError = nil
        } catch {
            sourceError = error.localizedDescription
            sourceStatus = "Stopped"
        }
    }

    private func applyPreviewMode() {
        session.localWantsColor = previewMode == .color
        session.localWantsDepth = previewMode == .depth
        session.localWantsDepthPreview = previewMode == .depth
        session.localColorPreviewFitsCamera = previewMode == .color && showsWholeCamera
        previewer.mode = previewMode
    }

    func connect() {
        let trimmed = host.trimmingCharacters(in: .whitespaces)
        guard !trimmed.isEmpty else { return }
        session.sender.connect(host: trimmed, port: UInt16(port) ?? L2K.defaultPort)
    }

    func connect(to receiver: L2KDiscovery.Receiver) {
        session.sender.connect(to: receiver.endpoint, label: receiver.name)
    }

    func disconnect() {
        session.sender.disconnect()
    }
}

/// Turns frames into preview images on the capture queue, at most 15 times per second.
final class Previewer {
    /// Called on the main queue.
    var onPreview: ((CGImage?, CGFloat, l2k_skeleton_frame, Double, String, [String], Float?, l2k_skeleton_frame?) -> Void)?
    var context: CIContext?
    weak var source: L2KSource?

    private let lock = NSLock()
    private var _mode = SensorModel.PreviewMode.color
    var mode: SensorModel.PreviewMode {
        get { lock.lock(); defer { lock.unlock() }; return _mode }
        set { lock.lock(); _mode = newValue; lock.unlock() }
    }

    private let previewQueue = DispatchQueue(label: "l2k.preview", qos: .userInitiated)
    private var building = false
    private var lastPreview: TimeInterval = 0
    private var rateStart: TimeInterval = 0
    private var rateCount = 0
    private var rate: Double = 0

    func handle(_ frame: L2KFrame) {
        let now = ProcessInfo.processInfo.systemUptime
        rateCount += 1
        if now - rateStart >= 1 {
            rate = Double(rateCount) / (now - rateStart)
            rateStart = now
            rateCount = 0
        }
        // Only the frames the session made a picture for (its local rate) can be shown. Showing the ones in between
        // blanked the preview: it flashed.
        let mode = mode
        let hasPicture = mode == .color ? (frame.color != nil || frame.colorPreview != nil) : (frame.depth != nil || frame.depthPreview != nil)
        guard hasPicture else { return }
        // Building the picture must never hold up the capture queue: it happens on its own queue, and a frame
        // that arrives while the previous picture is still being built is not shown.
        lock.lock()
        let busy = building
        if !busy { building = true }
        lock.unlock()
        guard !busy else { return }
        lastPreview = now
        let context = context, rate = rate
        let status = source?.statusText ?? "", diagnostics = source?.diagnostics ?? []
        previewQueue.async { [weak self] in
            var image: CGImage?
            var zoom: CGFloat = 1
            switch mode {
            case .color:
                if let wide = frame.colorPreview {
                    image = context?.createCGImage(wide, from: wide.extent)
                    zoom = CGFloat(frame.colorPreviewZoom)
                } else if let color = frame.color {
                    image = context?.createCGImage(color, from: color.extent)
                }
            case .depth:
                // What the sensor reports when it has a depth map; else the image that would be sent (a drawn figure).
                if let raw = frame.depthPreview {
                    image = Self.rawDepthImage(raw)
                } else if let depth = frame.depth {
                    image = Self.depthImage(depth)
                }
            }
            let skeleton = frame.skeleton
            if let image { self?.snapshotIfAsked(image, zoom: zoom, skeleton: skeleton) }
            let distance = frame.realDistance
            let picture = frame.pictureSkeleton
            DispatchQueue.main.async { self?.onPreview?(image, zoom, skeleton, rate, status, diagnostics, distance, picture) }
            guard let self else { return }
            self.lock.lock()
            self.building = false
            self.lock.unlock()
        }
    }

    /// -l2kSnapshot 1: every two seconds, write the preview picture with the skeleton drawn on it to
    /// Documents/l2k_snapshot.jpg (for troubleshooting over the cable; nothing is written otherwise).
    private var lastSnapshot: TimeInterval = 0
    private func snapshotIfAsked(_ image: CGImage, zoom: CGFloat, skeleton: l2k_skeleton_frame) {
        guard UserDefaults.standard.bool(forKey: "l2kSnapshot") else { return }
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastSnapshot >= 2 else { return }
        lastSnapshot = now
        let width = 640, height = 480
        guard let context = CGContext(data: nil, width: width, height: height, bitsPerComponent: 8, bytesPerRow: width * 4,
                                      space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue) else { return }
        context.draw(image, in: CGRect(x: 0, y: 0, width: width, height: height))
        // Core Graphics has its origin at the bottom-left: flip to the picture's top-left coordinates.
        context.translateBy(x: 0, y: CGFloat(height))
        context.scaleBy(x: 1, y: -1)
        func point(_ p: l2k_vec3) -> CGPoint? {
            var u: Float = 0, v: Float = 0
            guard l2k_skeleton_to_depth_pixel(p, 320, 240, &u, &v) == 1 else { return nil }
            return CGPoint(x: (0.5 + (CGFloat(u) / 320 - 0.5) / zoom) * CGFloat(width), y: (0.5 + (CGFloat(v) / 240 - 0.5) / zoom) * CGFloat(height))
        }
        let leftJoints: Set<KinectJoint> = [.shoulderLeft, .elbowLeft, .wristLeft, .handLeft, .hipLeft, .kneeLeft, .ankleLeft, .footLeft]
        let rightJoints: Set<KinectJoint> = [.shoulderRight, .elbowRight, .wristRight, .handRight, .hipRight, .kneeRight, .ankleRight, .footRight]
        for (_, body) in skeleton.trackedSkeletons {
            context.setLineWidth(3)
            for (a, b) in KinectJoint.bones {
                guard let pa = point(body.position(of: a)), let pb = point(body.position(of: b)) else { continue }
                let color: CGColor = leftJoints.contains(b) ? CGColor(red: 1, green: 0.55, blue: 0, alpha: 1)
                    : rightJoints.contains(b) ? CGColor(red: 0, green: 0.45, blue: 1, alpha: 1) : CGColor(red: 0, green: 1, blue: 0.3, alpha: 1)
                context.setStrokeColor(color)
                context.move(to: pa)
                context.addLine(to: pb)
                context.strokePath()
            }
        }
        guard let out = context.makeImage(),
              let documents = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else { return }
        let url = documents.appendingPathComponent("l2k_snapshot.jpg")
        if let destination = CGImageDestinationCreateWithURL(url as CFURL, "public.jpeg" as CFString, 1, nil) {
            CGImageDestinationAddImage(destination, out, [kCGImageDestinationLossyCompressionQuality: 0.7] as CFDictionary)
            CGImageDestinationFinalize(destination)
        }
    }

    /// The raw depth map: grey by distance (near = bright) inside the Kinect's 0.8-4 m range, blue outside it
    /// (seen by the LiDAR but a Kinect would report nothing), green where the person mask is set, black = no reading.
    static func rawDepthImage(_ raw: L2KDepthPreview) -> CGImage? {
        let width = L2K.depthWidth, height = L2K.depthHeight
        var rgba = [UInt8](repeating: 255, count: width * height * 4)
        let hasMask = !raw.person.isEmpty
        for i in 0..<(width * height) {
            let d = raw.metres[i]
            var r: Float = 0, g: Float = 0, b: Float = 0
            if d > 0 {
                let level = max(0.15, min(1, 1 - (d - 0.3) / 5.0))
                if hasMask && raw.person[i] != 0 {
                    (r, g, b) = (0.25 * level, level, 0.4 * level)
                } else if d < 0.8 || d > 4.0 {
                    (r, g, b) = (0.2 * level, 0.35 * level, level)
                } else {
                    (r, g, b) = (level, level, level)
                }
            }
            rgba[i * 4] = UInt8(255 * r)
            rgba[i * 4 + 1] = UInt8(255 * g)
            rgba[i * 4 + 2] = UInt8(255 * b)
        }
        guard let provider = CGDataProvider(data: Data(rgba) as CFData) else { return nil }
        return CGImage(width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: width * 4,
                       space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    }

    /// Depth as grey (near = bright), player pixels tinted by player index.
    static func depthImage(_ depth: [UInt16]) -> CGImage? {
        let width = L2K.depthWidth, height = L2K.depthHeight
        let tints: [(Float, Float, Float)] = [(1, 1, 1), (0.3, 1, 0.5), (0.4, 0.6, 1), (1, 0.8, 0.3), (1, 0.4, 0.8), (0.4, 1, 1), (1, 0.5, 0.4)]
        var rgba = [UInt8](repeating: 255, count: width * height * 4)
        for i in 0..<(width * height) {
            let mm = Float(l2k_depth_mm(depth[i]))
            let player = Int(l2k_depth_player(depth[i]))
            let level: Float = mm == 0 ? 0 : max(0.15, 1 - (mm - 800) / 3600)
            let tint = tints[min(player, tints.count - 1)]
            let scale = player == 0 ? level * 0.6 : level
            rgba[i * 4] = UInt8(255 * scale * tint.0)
            rgba[i * 4 + 1] = UInt8(255 * scale * tint.1)
            rgba[i * 4 + 2] = UInt8(255 * scale * tint.2)
        }
        guard let provider = CGDataProvider(data: Data(rgba) as CFData) else { return nil }
        return CGImage(width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: width * 4,
                       space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    }
}
