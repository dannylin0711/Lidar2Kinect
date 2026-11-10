// One capture source -> pipeline -> (a) a local consumer and (b) the network sender.
// The Lidar2Kinect app uses both; a game port that reads the sensor in-process uses only `onFrame`.
import CoreImage
import Foundation
import L2KShared

public final class L2KSession {
    public let pipeline = L2KPipeline()
    public let sender = L2KSender()
    public private(set) var source: L2KSource?

    /// Every produced frame, on the source's queue. Keep the work short or hand it off.
    public var onFrame: ((L2KFrame) -> Void)?
    /// Images the local consumer wants in `onFrame` even when the receiver did not ask for them.
    public var localWantsDepth = false
    public var localWantsColor = false
    /// The local consumer wants `L2KFrame.depthPreview` (the raw depth map, for showing what the sensor reports).
    public var localWantsDepthPreview = false
    /// Above 1: the local consumer also gets `L2KFrame.colorPreview`, a view this many times wider than the Kinect's.
    public var localColorPreviewZoom: Float = 1
    /// Output rate in frames per second. A Kinect delivers 30.
    public var outputRate: Double = 30
    /// Name announced to receivers.
    public var deviceName = "Lidar2Kinect"
    /// JPEG quality used when the receiver does not ask for one.
    public var defaultJPEGQuality = 0.6

    /// How often per second the local-only images (`localWants…`) are produced.
    public var localRate: Double = 15

    private var lastOutput: TimeInterval = 0
    private var lastLocal: TimeInterval = 0

    public init() {}

    public func start(_ source: L2KSource) throws {
        stop()
        self.source = source
        lastOutput = 0
        var capabilities = L2K.streamSkeleton | L2K.streamDepth
        if source.providesImages { capabilities |= L2K.streamColor }
        sender.setIdentity(.init(name: deviceName, capabilities: capabilities,
                                 maxTracked: UInt8(source.maxTracked), sourceKind: source.kind))
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

    private func handle(_ sourceFrame: L2KSourceFrame) {
        guard let source else { return }
        // Thin a faster camera down to the output rate (60 fps capture -> every second frame).
        let interval = 1.0 / outputRate
        if lastOutput > 0, sourceFrame.timestamp - lastOutput < interval * 0.75 { return }
        lastOutput = sourceFrame.timestamp

        let wanted = sender.wanted
        let sendDepth = wanted.streams & L2K.streamDepth != 0
        let sendColor = wanted.streams & L2K.streamColor != 0
        // Images only the local consumer wants (a preview) are made at the local rate, not for every frame.
        let localDue = sourceFrame.timestamp - lastLocal >= (1 / localRate) * 0.9
        if localDue { lastLocal = sourceFrame.timestamp }
        let frame = pipeline.process(sourceFrame, kind: source.kind,
                                     wantDepth: sendDepth || (localWantsDepth && localDue),
                                     wantColor: sendColor || (localWantsColor && localDue),
                                     playersOnly: wanted.depthMode == UInt8(L2K_DEPTH_MODE_PLAYERS),
                                     wantDepthPreview: localWantsDepthPreview && localDue,
                                     colorPreviewZoom: (localWantsColor && localDue) ? localColorPreviewZoom : 1)
        if wanted.streams != 0 {
            var jpeg: Data?
            if sendColor, let color = frame.color {
                let quality = wanted.quality == 0 ? defaultJPEGQuality : Double(min(wanted.quality, 100)) / 100
                jpeg = pipeline.jpeg(color, quality: quality)
            }
            sender.send(skeleton: frame.skeleton, depth: sendDepth ? frame.depth : nil, jpeg: jpeg)
        }
        onFrame?(frame)
    }
}
