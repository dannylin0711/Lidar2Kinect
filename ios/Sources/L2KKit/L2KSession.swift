// One capture source -> pipeline -> (a) a local consumer and (b) the network sender.
// The Lidar2Kinect app uses both; a game port that reads the sensor in-process uses only `onFrame`.
import CoreImage
import Foundation
import L2KShared

public final class L2KSession {
    public let pipeline = L2KPipeline()
    public let sender = L2KSender()
    /// Records the source's camera frames when asked (troubleshooting).
    public let recorder = L2KRecorder()
    public private(set) var source: L2KSource?

    /// Every produced frame, on the source's queue. Keep the work short or hand it off.
    public var onFrame: ((L2KFrame) -> Void)?
    /// The same frame as soon as its skeleton is known, before its images are made (they are all nil here) and
    /// before `onFrame`. For a consumer that judges movement: the images (a person mask, the depth image, the colour
    /// picture) take several milliseconds that the skeleton does not have to wait for. On the source's queue.
    public var onSkeleton: ((L2KFrame) -> Void)?
    /// Images the local consumer wants in `onFrame` even when the receiver did not ask for them.
    public var localWantsDepth = false
    public var localWantsColor = false
    /// The local consumer wants `L2KFrame.depthPreview` (the raw depth map, for showing what the sensor reports).
    public var localWantsDepthPreview = false
    /// Above 1: the local consumer also gets `L2KFrame.colorPreview`, a view this many times wider than the Kinect's.
    public var localColorPreviewZoom: Float = 1
    /// The local consumer's `L2KFrame.colorPreview` shows the whole camera image (for a camera that sees more than the
    /// Kinect; replaces `localColorPreviewZoom`).
    public var localColorPreviewFitsCamera = false
    /// Output rate in frames per second when the receiver asks for none (CONFIG rate 0). A Kinect delivers 30. A
    /// receiver that asks for a rate gets it (up to 120), as far as the source keeps up; images stay at 30 at most.
    public var outputRate: Double = L2K.defaultRate
    /// Name announced to receivers.
    public var deviceName = "Lidar2Kinect"
    /// JPEG quality used when the receiver does not ask for one.
    public var defaultJPEGQuality = 0.6

    /// How often per second the local-only images (`localWants…`) are produced.
    public var localRate: Double = 15

    private var lastOutput: TimeInterval = 0
    private var lastLocal: TimeInterval = 0
    private var lastImages: TimeInterval = 0
    private var requestedRate: Double = 0
    private let settingsLock = NSLock()
    private var wantedColorEnabled = true

    /// Whether the camera picture (the Kinect's colour stream) may be sent to the receiver. Off: the sender no
    /// longer offers it, so only the skeleton and the depth image leave the device. Safe to set from any thread;
    /// takes effect at once for what is sent and within a second for what is announced.
    public var colorEnabled: Bool {
        get { settingsLock.lock(); defer { settingsLock.unlock() }; return wantedColorEnabled }
        set {
            settingsLock.lock()
            wantedColorEnabled = newValue
            settingsLock.unlock()
            if let source { announce(source) }
        }
    }

    /// The streams a source can offer, given the colour setting.
    static func capabilities(providesImages: Bool, colorEnabled: Bool) -> UInt32 {
        var capabilities = L2K.streamSkeleton | L2K.streamBody | L2K.streamDepth
        if providesImages && colorEnabled { capabilities |= L2K.streamColor }
        return capabilities
    }

    private func announce(_ source: L2KSource) {
        sender.setIdentity(.init(name: deviceName,
                                 capabilities: Self.capabilities(providesImages: source.providesImages, colorEnabled: colorEnabled),
                                 maxTracked: UInt8(source.maxTracked), sourceKind: source.kind))
    }

    public init() {}

    public func start(_ source: L2KSource) throws {
        stop()
        self.source = source
        lastOutput = 0
        lastImages = 0
        requestedRate = 0
        announce(source)
        source.onFrame = { [weak self] frame in self?.handle(frame) }
        do {
            try source.start()
        } catch {
            source.onFrame = nil
            self.source = nil
            throw error
        }
    }

    public func stop() {
        source?.stop()
        source?.onFrame = nil
        source = nil
    }

    /// The skeleton rate for what the receiver wants: its CONFIG rate, else `outputRate`.
    static func frameRate(asked: UInt8, otherwise: Double) -> Double {
        asked > 0 ? min(Double(asked), 120) : otherwise
    }

    private func handle(_ sourceFrame: L2KSourceFrame) {
        guard let source else { return }
        let wanted = sender.wanted
        // The receiver's game decides the play area and the standing point. Kept while no receiver is there, so a
        // short silence does not move the player.
        if wanted.streams != 0 { pipeline.game = L2KGame(consumerKind: wanted.consumerKind) }
        // Thin a faster camera down to the output rate (60 fps capture -> every second frame, unless 60 is asked for).
        let rate = Self.frameRate(asked: wanted.streams != 0 ? wanted.rate : 0, otherwise: outputRate)
        if rate != requestedRate {
            requestedRate = rate
            source.requestFrameRate(rate)
        }
        let interval = 1.0 / rate
        if lastOutput > 0, sourceFrame.timestamp - lastOutput < interval * 0.75 { return }
        lastOutput = sourceFrame.timestamp

        recorder.append(sourceFrame.image, timestamp: sourceFrame.timestamp, extra: [
            "up": [sourceFrame.upCamera.x, sourceFrame.upCamera.y, sourceFrame.upCamera.z],
            "quadrant": Int(l2k_quadrant_from_up(sourceFrame.upCamera.x, sourceFrame.upCamera.y)),
            "camera": [sourceFrame.camera.fxn, sourceFrame.camera.fyn, sourceFrame.camera.cxn, sourceFrame.camera.cyn],
        ])
        // Images at the Kinect's rate at most, whatever the skeleton's rate.
        let imagesDue = lastImages == 0 || sourceFrame.timestamp - lastImages >= (1 / L2K.defaultRate) * 0.75
        if imagesDue { lastImages = sourceFrame.timestamp }
        let sendDepth = imagesDue && wanted.streams & L2K.streamDepth != 0
        let sendColor = imagesDue && wanted.streams & L2K.streamColor != 0 && colorEnabled
        // Images only the local consumer wants (a preview) are made at the local rate, not for every frame.
        let localDue = sourceFrame.timestamp - lastLocal >= (1 / localRate) * 0.9
        if localDue { lastLocal = sourceFrame.timestamp }
        // The skeleton first: it goes to the receiver and to the local consumer before any image is made.
        var frame = pipeline.skeletonFrame(sourceFrame, kind: source.kind)
        frame.forLocal = localDue
        if wanted.streams != 0 { sender.send(skeleton: frame.skeleton, body: wanted.streams & L2K.streamBody != 0 ? frame.body : nil) }
        onSkeleton?(frame)
        pipeline.addImages(to: &frame, from: sourceFrame,
                           wantDepth: sendDepth || (localWantsDepth && localDue),
                           wantColor: sendColor || (localWantsColor && localDue),
                           playersOnly: wanted.depthMode == UInt8(L2K_DEPTH_MODE_PLAYERS),
                           wantDepthPreview: localWantsDepthPreview && localDue,
                           colorPreviewZoom: (localWantsColor && localDue) ? localColorPreviewZoom : 1,
                           colorPreviewFit: localWantsColor && localDue && localColorPreviewFitsCamera)
        if wanted.streams != 0, sendDepth || sendColor {
            var jpeg: Data?
            if sendColor, let color = frame.color {
                let quality = wanted.quality == 0 ? defaultJPEGQuality : Double(min(wanted.quality, 100)) / 100
                jpeg = pipeline.jpeg(color, quality: quality)
            }
            sender.sendImages(of: frame.skeleton, depth: sendDepth ? frame.depth : nil, jpeg: jpeg)
        }
        onFrame?(frame)
    }
}
