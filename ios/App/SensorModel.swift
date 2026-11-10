// App state: owns the capture session, the sender and the receiver discovery, and feeds the UI.
import CoreImage
import L2KKit
import L2KShared
import SwiftUI

@MainActor
final class SensorModel: ObservableObject {
    enum SourceChoice: String, CaseIterable, Identifiable {
        case lidar, arkit, synthetic
        var id: String { rawValue }
        var title: String {
            switch self {
            case .lidar: return "LiDAR depth"
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
    @Published var sourceStatus = "Not started"
    @Published var sourceError: String?
    @Published var senderStatus = L2KSender.Status()
    @Published var receivers: [L2KDiscovery.Receiver] = []
    @Published var preview: CGImage?
    @Published var skeleton: l2k_skeleton_frame?
    @Published var frameRate: Double = 0
    @Published var diagnostics: [String] = []
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
        if ARKitBodySource.isSupported { sources.append(.arkit) }
        deviceSupport = L2KDeviceSupport.report()
        #endif
        sources.append(.synthetic)
        availableSources = sources
        // Launch arguments (also plain defaults): -l2kSource lidar|arkit|synthetic, -l2kHost, -l2kPort, -l2kPlayers, -l2kAutoConnect 1
        // Default: the first camera source the device supports (LiDAR depth before 3D body tracking).
        let wanted = SourceChoice(rawValue: defaults.string(forKey: "l2kSource") ?? "")
        sourceChoice = wanted.flatMap { sources.contains($0) ? $0 : nil } ?? sources[0]
        host = defaults.string(forKey: "l2kHost") ?? ""
        port = defaults.string(forKey: "l2kPort") ?? String(L2K.defaultPort)
        players = max(1, defaults.integer(forKey: "l2kPlayers"))
        upperBodyOnly = defaults.bool(forKey: "l2kUpperBody") // also a launch argument: -l2kUpperBody 1
        session.pipeline.upperBodyMode = upperBodyOnly
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
        previewer.onPreview = { [weak self] image, skeleton, rate, status, diagnostics in
            guard let self else { return }
            self.preview = image
            self.skeleton = skeleton
            self.frameRate = rate
            self.sourceStatus = status
            self.diagnostics = diagnostics
        }
        applyPreviewMode()
        startSource()
        if UserDefaults.standard.bool(forKey: "l2kAutoConnect"), !host.isEmpty { connect() }
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
        case .lidar: source = VisionDepthSource()
        case .arkit: source = ARKitBodySource()
        case .synthetic: source = SyntheticSource(players: players)
        }
        #endif
        previewer.source = source
        previewer.context = session.pipeline.ciContext
        session.onFrame = { [previewer] frame in previewer.handle(frame) }
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
    var onPreview: ((CGImage?, l2k_skeleton_frame, Double, String, [String]) -> Void)?
    var context: CIContext?
    weak var source: L2KSource?

    private let lock = NSLock()
    private var _mode = SensorModel.PreviewMode.color
    var mode: SensorModel.PreviewMode {
        get { lock.lock(); defer { lock.unlock() }; return _mode }
        set { lock.lock(); _mode = newValue; lock.unlock() }
    }

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
        guard now - lastPreview >= 1.0 / 15 else { return }
        lastPreview = now
        var image: CGImage?
        switch mode {
        case .color:
            if let color = frame.color { image = context?.createCGImage(color, from: color.extent) }
        case .depth:
            // What the sensor reports when it has a depth map; else the image that would be sent (a drawn figure).
            if let raw = frame.depthPreview {
                image = Self.rawDepthImage(raw)
            } else if let depth = frame.depth {
                image = Self.depthImage(depth)
            }
        }
        let skeleton = frame.skeleton, rate = rate, status = source?.statusText ?? "", diagnostics = source?.diagnostics ?? []
        DispatchQueue.main.async { [weak self] in self?.onPreview?(image, skeleton, rate, status, diagnostics) }
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
