// A source without a camera: the shared library's dancing figure, plus a generated colour image.
// It runs everywhere (simulator, macOS tests) and exercises the whole pipeline and the wire protocol.
import CoreVideo
import Foundation
import L2KShared

public final class SyntheticSource: L2KSource {
    public let kind = UInt8(L2K_SOURCE_SYNTHETIC)
    public var maxTracked: Int { players }
    public let providesImages = true
    public var statusText: String { "Synthetic figure, \(players) player\(players == 1 ? "" : "s"), \(Int(fps)) fps" }
    public var onFrame: ((L2KSourceFrame) -> Void)?

    public let players: Int
    public let fps: Double
    private let queue = DispatchQueue(label: "l2k.source.synthetic")
    private var timer: DispatchSourceTimer?
    private var startTime: TimeInterval = 0

    public init(players: Int = 1, fps: Double = 30) {
        self.players = max(1, min(2, players))
        self.fps = fps
    }

    public func start() throws {
        stop()
        startTime = ProcessInfo.processInfo.systemUptime
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now(), repeating: 1.0 / fps, leeway: .milliseconds(1))
        timer.setEventHandler { [weak self] in
            guard let self else { return }
            let now = ProcessInfo.processInfo.systemUptime
            self.onFrame?(self.frame(at: now - self.startTime, timestamp: now))
        }
        self.timer = timer
        timer.resume()
    }

    public func stop() {
        timer?.cancel()
        timer = nil
    }

    /// The frame for scene time `t` (seconds). Deterministic; also used directly by tests.
    public func frame(at t: TimeInterval, timestamp: TimeInterval) -> L2KSourceFrame {
        var bodies: [l2k_src_body] = []
        for player in 0..<players {
            var body = l2k_src_body()
            l2k_synth_body(t, Int32(player), Int32(players), &body)
            bodies.append(body)
        }
        // A camera identical to the virtual Kinect: the colour image passes through, mirrored only.
        let camera = l2k_src_camera(fxn: L2K_FOCAL_320 / 320, fyn: L2K_FOCAL_320 / 240, cxn: 0.5, cyn: 0.5)
        return L2KSourceFrame(timestamp: timestamp, bodies: bodies,
                              upCamera: SIMD3(L2K_SYNTH_UP_X, L2K_SYNTH_UP_Y, L2K_SYNTH_UP_Z), camera: camera,
                              image: Self.testImage(at: t))
    }

    /// The still part of the test picture (BGRA, tightly packed): colour bars and a red corner mark.
    private static let template: [UInt8] = {
        let width = L2K.colorWidth, height = L2K.colorHeight
        let bars: [(UInt8, UInt8, UInt8)] = [(40, 40, 40), (60, 20, 20), (20, 60, 20), (20, 20, 60), (60, 60, 20), (20, 60, 60), (60, 20, 60), (70, 70, 70)]
        var pixels = [UInt8](repeating: 255, count: width * height * 4)
        for y in 0..<height {
            for x in 0..<width {
                var (r, g, b) = bars[x * bars.count / width]
                if y < 8 && x < 8 { (r, g, b) = (255, 0, 0) } // marks the source image's top-left corner
                pixels[(y * width + x) * 4] = b
                pixels[(y * width + x) * 4 + 1] = g
                pixels[(y * width + x) * 4 + 2] = r
            }
        }
        return pixels
    }()

    /// 640x480 BGRA test picture: the template with a white bar that sweeps across once every two seconds.
    static func testImage(at t: TimeInterval) -> CVPixelBuffer? {
        let width = L2K.colorWidth, height = L2K.colorHeight
        var buffer: CVPixelBuffer?
        let attributes: [CFString: Any] = [kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary]
        guard CVPixelBufferCreate(nil, width, height, kCVPixelFormatType_32BGRA, attributes as CFDictionary, &buffer) == kCVReturnSuccess,
              let buffer else { return nil }
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { return nil }
        let stride = CVPixelBufferGetBytesPerRow(buffer)
        let sweep = Int((t.truncatingRemainder(dividingBy: 2.0) / 2.0) * Double(width))
        let x0 = max(0, sweep - 5), x1 = min(width - 1, sweep + 5)
        template.withUnsafeBytes { source in
            for y in 0..<height {
                let row = base.advanced(by: y * stride)
                row.copyMemory(from: source.baseAddress!.advanced(by: y * width * 4), byteCount: width * 4)
                if x0 <= x1 { memset(row.advanced(by: x0 * 4), 255, (x1 - x0 + 1) * 4) }
            }
        }
        return buffer
    }
}
