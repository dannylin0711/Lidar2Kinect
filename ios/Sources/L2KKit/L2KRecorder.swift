// Records the source camera's frames to a movie file for a few seconds, for troubleshooting: the clip is run
// through the detectors again on the Mac to measure what they find. Nothing is recorded unless asked.
import AVFoundation
import CoreVideo
import Foundation

public final class L2KRecorder {
    public enum State: Equatable {
        case idle
        case recording(secondsLeft: Int)
        case saved(String)
        case failed(String)
    }

    /// Called on the main queue when the state changes.
    public var onState: ((State) -> Void)?
    public private(set) var state: State = .idle {
        didSet {
            guard state != oldValue else { return }
            let state = state
            DispatchQueue.main.async { self.onState?(state) }
        }
    }

    private let lock = NSLock()
    private var writer: AVAssetWriter?
    private var input: AVAssetWriterInput?
    private var adaptor: AVAssetWriterInputPixelBufferAdaptor?
    private var url: URL?
    private var wanted: TimeInterval = 0
    private var firstTime: TimeInterval?
    private var armed = false
    private var notes: [String: Any] = [:]

    public init() {}

    /// Record the next `seconds` of frames to `url` (replaced if it exists). `notes` goes into a JSON file next to it.
    public func start(url: URL, seconds: TimeInterval, notes: [String: Any]) {
        lock.lock()
        defer { lock.unlock() }
        guard !armed else { return }
        self.url = url
        self.wanted = seconds
        self.notes = notes
        self.firstTime = nil
        self.writer = nil
        armed = true
        state = .recording(secondsLeft: Int(seconds.rounded(.up)))
    }

    /// Give every source frame to the recorder; it ignores them unless a recording is running.
    public func append(_ image: CVPixelBuffer?, timestamp: TimeInterval, extra: [String: Any] = [:]) {
        lock.lock()
        defer { lock.unlock() }
        guard armed, let image, let url else { return }
        if writer == nil {
            try? FileManager.default.removeItem(at: url)
            guard let writer = try? AVAssetWriter(outputURL: url, fileType: .mov) else {
                armed = false
                state = .failed("The movie file could not be created")
                return
            }
            let settings: [String: Any] = [
                AVVideoCodecKey: AVVideoCodecType.h264,
                AVVideoWidthKey: CVPixelBufferGetWidth(image),
                AVVideoHeightKey: CVPixelBufferGetHeight(image),
            ]
            let input = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
            input.expectsMediaDataInRealTime = true
            let adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input, sourcePixelBufferAttributes: nil)
            guard writer.canAdd(input) else {
                armed = false
                state = .failed("The movie input could not be added")
                return
            }
            writer.add(input)
            writer.startWriting()
            writer.startSession(atSourceTime: .zero)
            self.writer = writer
            self.input = input
            self.adaptor = adaptor
            firstTime = timestamp
            for (key, value) in extra { notes[key] = value }
            notes["width"] = CVPixelBufferGetWidth(image)
            notes["height"] = CVPixelBufferGetHeight(image)
        }
        guard let first = firstTime, let input, let adaptor, let writer else { return }
        let t = timestamp - first
        if t >= wanted {
            armed = false
            input.markAsFinished()
            let notes = notes, url = url
            writer.finishWriting { [weak self] in
                if let data = try? JSONSerialization.data(withJSONObject: notes, options: [.prettyPrinted, .sortedKeys]) {
                    try? data.write(to: url.deletingPathExtension().appendingPathExtension("json"))
                }
                self?.state = writer.status == .completed ? .saved(url.lastPathComponent) : .failed(writer.error?.localizedDescription ?? "The movie was not saved")
            }
            return
        }
        if input.isReadyForMoreMediaData {
            adaptor.append(image, withPresentationTime: CMTime(seconds: t, preferredTimescale: 600))
        }
        state = .recording(secondsLeft: max(0, Int((wanted - t).rounded(.up))))
    }
}
