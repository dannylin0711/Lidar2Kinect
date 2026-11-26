// What a capture source hands to the pipeline, and the protocol every source implements.
import CoreVideo
import Foundation
import L2KShared

/// One capture instant, in the source camera's own terms. The pipeline turns it into Kinect data.
public struct L2KSourceFrame {
    /// Capture time in seconds on the system uptime clock (ARFrame.timestamp's clock).
    public var timestamp: TimeInterval
    /// Bodies in source camera space (+x image right, +y image up, +z towards the viewer), metres.
    public var bodies: [l2k_src_body]
    /// World up (opposite to gravity) in source camera space.
    public var upCamera: SIMD3<Float>
    /// The camera's pinhole model, normalised by the image size.
    public var camera: l2k_src_camera
    /// Depth along the optical axis in metres (kCVPixelFormatType_DepthFloat32), registered to `image`.
    public var depthMap: CVPixelBuffer?
    /// Person segmentation (kCVPixelFormatType_OneComponent8), registered to `image`.
    public var segmentation: CVPixelBuffer?
    /// A source whose mask costs time gives this instead of `segmentation`: the mask is then made only when an image
    /// needs it, after the skeleton has been handed on (`personMask`). Called on the source's queue, inside `onFrame`.
    public var makeSegmentation: (() -> CVPixelBuffer?)?
    /// The camera image.
    public var image: CVPixelBuffer?

    public init(timestamp: TimeInterval, bodies: [l2k_src_body], upCamera: SIMD3<Float>, camera: l2k_src_camera,
                depthMap: CVPixelBuffer? = nil, segmentation: CVPixelBuffer? = nil, image: CVPixelBuffer? = nil) {
        self.timestamp = timestamp
        self.bodies = bodies
        self.upCamera = upCamera
        self.camera = camera
        self.depthMap = depthMap
        self.segmentation = segmentation
        self.image = image
    }

    /// The person mask: the one delivered with the frame, else the one the source makes when asked.
    public var personMask: CVPixelBuffer? { segmentation ?? makeSegmentation?() }
}

public enum L2KSourceError: LocalizedError {
    case unsupported(String)

    public var errorDescription: String? {
        switch self {
        case .unsupported(let reason): return reason
        }
    }
}

/// A body capture source. `onFrame` is called on the source's own queue, never the main queue.
public protocol L2KSource: AnyObject {
    /// L2K_SOURCE_* of the shared library.
    var kind: UInt8 { get }
    /// Bodies it can fully track at once.
    var maxTracked: Int { get }
    /// True when frames carry a camera image, so a colour stream can be produced.
    var providesImages: Bool { get }
    /// One line for the UI: what the source is doing / what it lacks.
    var statusText: String { get }
    /// Detail lines for troubleshooting (what the sensors deliver right now). May be empty.
    var diagnostics: [String] { get }
    var onFrame: ((L2KSourceFrame) -> Void)? { get set }
    func start() throws
    func stop()
    /// Frames per second the source can deliver at most as it runs now (a camera's capture rate).
    var maxFrameRate: Double { get }
    /// Deliver up to this many frames a second (30 by default; a receiver can ask for 60). A source that thins a faster
    /// camera down keeps more of its frames. Called on the source's own queue, from inside `onFrame`.
    func requestFrameRate(_ rate: Double)
}

public extension L2KSource {
    var diagnostics: [String] { [] }
    var maxFrameRate: Double { L2K.defaultRate }
    func requestFrameRate(_ rate: Double) {}
}
