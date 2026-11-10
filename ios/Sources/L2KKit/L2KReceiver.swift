// The L2K1 receiver: listens for a sender (the Lidar2Kinect app on another device), answers its HELLO with CONFIG and
// turns the streams it sends back into `L2KFrame`s - what a game port consumes in place of an in-process source.
// Protocol rules: docs/02-wire-protocol.md. Packet bytes are read and written by the shared C library.
import CoreImage
import Foundation
import L2KShared
import Network

public final class L2KReceiver {
    public enum State: Equatable {
        case idle
        case listening          // the port is open, no sender is talking to it
        case receiving          // a sender said HELLO within the last L2K_PEER_TIMEOUT_MS
        case failed(String)
    }

    public struct Status: Equatable {
        public var state: State = .idle
        /// The UDP port actually listened on.
        public var port: UInt16 = 0
        public var senderName = ""
        public var senderAddress = ""
        /// L2K_SOURCE_* the sender announced.
        public var sourceKind: UInt8 = 0
        /// L2K_STREAM_* asked of this sender (what was wanted and it can produce).
        public var streams: UInt32 = 0
        public var skeletonFrames = 0
        public var depthFrames = 0
        public var colorFrames = 0
        /// Skeleton frames the sender produced that never arrived (gaps in the frame numbers).
        public var lostFrames = 0

        public init() {}
    }

    /// Every skeleton frame received, on the receiver's queue. `depth` / `color` are set on the first frame after
    /// a whole image arrived and nil otherwise. Keep the work short or hand it off.
    public var onFrame: ((L2KFrame) -> Void)?
    /// Called on the receiver's queue when the status changes, and once a second while receiving.
    public var onStatus: ((Status) -> Void)?

    // What to ask a sender for. Set these before `start`.
    public var streams: UInt32 = L2K.streamSkeleton | L2K.streamDepth
    public var depthMode = UInt8(L2K_DEPTH_MODE_PLAYERS)
    /// JPEG quality 1...100 for the colour stream, 0 = the sender's default.
    public var colorQuality: UInt8 = 0
    /// Name shown on the sending device, and the Bonjour service name.
    public var name = "Game"
    public var consumerKind = UInt8(L2K_CONSUMER_GAME_PORT)

    private let queue = DispatchQueue(label: "l2k.receiver")
    private var listener: NWListener?
    private var connections: [ObjectIdentifier: NWConnection] = [:]
    private var active: NWConnection?
    private var activeSession: UInt32 = 0
    private var lastPacket: TimeInterval = 0
    private var lastFrameNumber: UInt32?
    private var timer: DispatchSourceTimer?
    private let session = UInt32.random(in: 1...UInt32.max)
    private var seq: UInt32 = 0
    private var status = Status()

    private static let depthCapacity = max(Int(l2k_depth_d1_bound(L2K.depthWidth * L2K.depthHeight)), L2K.depthWidth * L2K.depthHeight * 2)
    private static let colorCapacity = Int(L2K_MAX_CHUNKS) * Int(L2K_CHUNK_PAYLOAD)
    private let depthStorage = UnsafeMutablePointer<UInt8>.allocate(capacity: L2KReceiver.depthCapacity)
    private let colorStorage = UnsafeMutablePointer<UInt8>.allocate(capacity: L2KReceiver.colorCapacity)
    private var depthAssembler = l2k_assembler()
    private var colorAssembler = l2k_assembler()
    private var pendingDepth: [UInt16]?
    private var pendingColor: CIImage?

    public init() {
        resetAssemblers()
    }

    deinit {
        listener?.cancel()
        timer?.cancel()
        depthStorage.deallocate()
        colorStorage.deallocate()
    }

    /// Open the port (0 = any free port; `Status.port` says which) and, with `advertise`, announce it with Bonjour so
    /// the sending app lists this receiver by name.
    public func start(port: UInt16 = L2K.defaultPort, advertise: Bool = true) {
        queue.async { [weak self] in
            guard let self else { return }
            self.teardown()
            do {
                let parameters = NWParameters.udp
                parameters.allowLocalEndpointReuse = true
                let listener: NWListener
                if port == 0 {
                    listener = try NWListener(using: parameters)
                } else if let nwPort = NWEndpoint.Port(rawValue: port) {
                    listener = try NWListener(using: parameters, on: nwPort)
                } else {
                    throw NWError.posix(.EINVAL)
                }
                if advertise { listener.service = NWListener.Service(name: self.name, type: L2K.bonjourType) }
                listener.stateUpdateHandler = { [weak self, weak listener] state in
                    guard let self, let listener, self.listener === listener else { return }
                    switch state {
                    case .ready:
                        self.status.port = listener.port?.rawValue ?? port
                        if self.active == nil { self.status.state = .listening }
                        self.publish()
                    case .waiting(let error), .failed(let error):
                        self.status.state = .failed(error.localizedDescription)
                        self.publish()
                    default:
                        break
                    }
                }
                listener.newConnectionHandler = { [weak self] connection in self?.accept(connection) }
                self.listener = listener
                listener.start(queue: self.queue)
                self.startTimer()
            } catch {
                self.status.state = .failed(error.localizedDescription)
                self.publish()
            }
        }
    }

    public func stop() {
        queue.async { [weak self] in
            guard let self else { return }
            if let active = self.active {
                var packet = [UInt8](repeating: 0, count: Int(L2K_HEADER_SIZE))
                self.seq &+= 1
                let n = l2k_encode_bye(&packet, packet.count, self.session, self.seq)
                if n > 0 { active.send(content: Data(packet[0..<n]), completion: .idempotent) }
            }
            self.teardown()
            self.status = Status()
            self.publish()
        }
    }

    // MARK: - on `queue`

    private func teardown() {
        timer?.cancel()
        timer = nil
        listener?.cancel()
        listener = nil
        for connection in connections.values { connection.cancel() }
        connections.removeAll()
        active = nil
        lastFrameNumber = nil
        pendingDepth = nil
        pendingColor = nil
        resetAssemblers()
    }

    private func resetAssemblers() {
        l2k_assembler_init(&depthAssembler, depthStorage, Self.depthCapacity)
        l2k_assembler_init(&colorAssembler, colorStorage, Self.colorCapacity)
    }

    private func publish() {
        onStatus?(status)
    }

    private func startTimer() {
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + 1, repeating: 1)
        timer.setEventHandler { [weak self] in
            guard let self, self.active != nil else { return }
            if ProcessInfo.processInfo.systemUptime - self.lastPacket > Double(L2K_PEER_TIMEOUT_MS) / 1000 {
                self.senderGone()
            } else {
                self.publish()
            }
        }
        timer.resume()
        self.timer = timer
    }

    private func senderGone() {
        active = nil
        lastFrameNumber = nil
        pendingDepth = nil
        pendingColor = nil
        resetAssemblers()
        status.state = listener == nil ? .idle : .listening
        status.streams = 0
        publish()
    }

    private func accept(_ connection: NWConnection) {
        let id = ObjectIdentifier(connection)
        connections[id] = connection
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection else { return }
            switch state {
            case .failed, .cancelled:
                self.connections[ObjectIdentifier(connection)] = nil
                if self.active === connection { self.senderGone() }
            default:
                break
            }
        }
        connection.start(queue: queue)
        receive(on: connection)
    }

    private func receive(on connection: NWConnection) {
        connection.receiveMessage { [weak self, weak connection] data, _, _, error in
            guard let self, let connection else { return }
            if let data, !data.isEmpty { self.handle(data, from: connection) }
            if error == nil {
                self.receive(on: connection)
            } else {
                connection.cancel()
            }
        }
    }

    private func handle(_ data: Data, from connection: NWConnection) {
        data.withUnsafeBytes { raw in
            guard let bytes = raw.bindMemory(to: UInt8.self).baseAddress else { return }
            var header = l2k_header()
            guard l2k_decode_header(bytes, raw.count, &header) == L2K_OK else { return }
            let now = ProcessInfo.processInfo.systemUptime
            let type = Int32(header.type)

            if type == Int32(L2K_PKT_HELLO) {
                var hello = l2k_hello()
                guard l2k_decode_hello(bytes, raw.count, &hello) == L2K_OK else { return }
                // One sender at a time: a second device is ignored while the first one is alive.
                let free = active == nil || now - lastPacket > Double(L2K_PEER_TIMEOUT_MS) / 1000
                guard free || active === connection else { return }
                if active !== connection || header.session != activeSession {
                    active = connection
                    activeSession = header.session
                    lastFrameNumber = nil
                    pendingDepth = nil
                    pendingColor = nil
                    resetAssemblers()
                }
                lastPacket = now
                var config = l2k_config()
                config.echo_sender_time_us = hello.sender_time_us
                config.receiver_time_us = L2K.nowMicroseconds()
                config.streams = streams & hello.caps
                config.consumer_kind = consumerKind
                config.depth_mode = depthMode
                config.color_quality = colorQuality
                l2kSetName(&config.name, name)
                var packet = [UInt8](repeating: 0, count: 128)
                seq &+= 1
                let n = l2k_encode_config(&packet, packet.count, session, seq, &config)
                if n > 0 { connection.send(content: Data(packet[0..<n]), completion: .idempotent) }
                let before = status
                status.state = .receiving
                status.senderName = l2kGetName(hello.name)
                status.senderAddress = "\(connection.endpoint)"
                status.sourceKind = hello.source_kind
                status.streams = config.streams
                if status != before { publish() }
                return
            }

            guard active === connection else { return }
            if header.session != activeSession { // the sending program restarted without a BYE
                activeSession = header.session
                lastFrameNumber = nil
                resetAssemblers()
            }
            lastPacket = now

            switch type {
            case Int32(L2K_PKT_BYE):
                senderGone()
            case Int32(L2K_PKT_SKELETON):
                var frame = l2k_skeleton_frame()
                guard l2k_decode_skeleton(bytes, raw.count, &frame) == L2K_OK else { return }
                if let last = lastFrameNumber {
                    guard l2k_seq_newer(frame.frame_number, last) != 0 else { return } // late or repeated datagram
                    status.lostFrames += Int(frame.frame_number &- last) - 1
                }
                lastFrameNumber = frame.frame_number
                status.skeletonFrames += 1
                let out = L2KFrame(skeleton: frame, depth: pendingDepth, color: pendingColor, quadrant: 0)
                pendingDepth = nil
                pendingColor = nil
                onFrame?(out)
            case Int32(L2K_PKT_DEPTH):
                var chunk = l2k_chunk()
                guard l2k_decode_chunk(bytes, raw.count, &chunk) == L2K_OK,
                      l2k_assembler_push(&depthAssembler, &chunk) == 1 else { return }
                let info = depthAssembler.info
                let count = L2K.depthWidth * L2K.depthHeight
                guard Int(info.width) == L2K.depthWidth, Int(info.height) == L2K.depthHeight else { return }
                var pixels = [UInt16](repeating: 0, count: count)
                if Int32(info.encoding) == Int32(L2K_DEPTH_D1) {
                    guard l2k_depth_d1_decode(depthStorage, Int(info.total_len), &pixels, count) == 0 else { return }
                } else {
                    guard Int(info.total_len) == count * 2 else { return }
                    for i in 0..<count { pixels[i] = UInt16(depthStorage[2 * i]) | UInt16(depthStorage[2 * i + 1]) << 8 }
                }
                pendingDepth = pixels
                status.depthFrames += 1
            case Int32(L2K_PKT_COLOR):
                var chunk = l2k_chunk()
                guard l2k_decode_chunk(bytes, raw.count, &chunk) == L2K_OK,
                      l2k_assembler_push(&colorAssembler, &chunk) == 1 else { return }
                let info = colorAssembler.info
                guard Int32(info.encoding) == Int32(L2K_COLOR_JPEG),
                      let image = CIImage(data: Data(bytes: colorStorage, count: Int(info.total_len))) else { return }
                pendingColor = image
                status.colorFrames += 1
            default:
                break
            }
        }
    }
}
