// App state: owns the capture session, the sender and the receiver discovery, and feeds the UI.
import CoreImage
import L2KKit
import L2KShared
import SwiftUI

@MainActor
final class SensorModel: ObservableObject {
    enum SourceChoice: String, CaseIterable, Identifiable {
        case arkit, synthetic
        var id: String { rawValue }
        var title: String { self == .arkit ? "Camera + LiDAR" : "Test figure" }
    }

    enum PreviewMode: String, CaseIterable, Identifiable {
        case color, depth
        var id: String { rawValue }
        var title: String { self == .color ? "Colour" : "Depth" }
    }

    @Published var sourceChoice: SourceChoice
    @Published var previewMode: PreviewMode = .color { didSet { applyPreviewMode() } }
    @Published var sourceStatus = "Not started"
    @Published var sourceError: String?
    @Published var senderStatus = L2KSender.Status()
    @Published var receivers: [L2KDiscovery.Receiver] = []
    @Published var preview: CGImage?
    @Published var skeleton: l2k_skeleton_frame?
    @Published var frameRate: Double = 0
    @Published var host: String { didSet { UserDefaults.standard.set(host, forKey: "l2kHost") } }
    @Published var port: String { didSet { UserDefaults.standard.set(port, forKey: "l2kPort") } }

    /// True when this device can run the camera source at all (false in the simulator).
    let cameraAvailable: Bool
    private let session = L2KSession()
    private let discovery = L2KDiscovery()
    private let previewer = Previewer()
    private var started = false
    private let players: Int

    init() {
        let defaults = UserDefaults.standard
        #if targetEnvironment(simulator)
        cameraAvailable = false
        #else
        cameraAvailable = ARKitBodySource.isSupported
        #endif
        // Launch arguments (also plain defaults): -l2kSource synthetic|arkit, -l2kHost, -l2kPort, -l2kPlayers, -l2kAutoConnect 1
        let wanted = SourceChoice(rawValue: defaults.string(forKey: "l2kSource") ?? "") ?? .arkit
        sourceChoice = cameraAvailable ? wanted : .synthetic
        host = defaults.string(forKey: "l2kHost") ?? ""
        port = defaults.string(forKey: "l2kPort") ?? String(L2K.defaultPort)
        players = max(1, defaults.integer(forKey: "l2kPlayers"))
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
        previewer.onPreview = { [weak self] image, skeleton, rate, status in
            guard let self else { return }
            self.preview = image
            self.skeleton = skeleton
            self.frameRate = rate
            self.sourceStatus = status
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
        source = sourceChoice == .arkit ? ARKitBodySource() : SyntheticSource(players: players)
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
    var onPreview: ((CGImage?, l2k_skeleton_frame, Double, String) -> Void)?
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
            if let depth = frame.depth { image = Self.depthImage(depth) }
        }
        let skeleton = frame.skeleton, rate = rate, status = source?.statusText ?? ""
        DispatchQueue.main.async { [weak self] in self?.onPreview?(image, skeleton, rate, status) }
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
