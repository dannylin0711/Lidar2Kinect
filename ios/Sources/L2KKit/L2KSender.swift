// The L2K1 sender: HELLO / CONFIG handshake with one receiver over UDP, then the streams it asks for.
// Protocol rules: docs/02-wire-protocol.md. Packet bytes come from the shared C library.
import Foundation
import L2KShared
import Network

public final class L2KSender {
    public enum State: Equatable {
        case idle
        case connecting            // the UDP path is not ready yet
        case waitingForReceiver    // sending HELLO, no CONFIG (yet / any more)
        case streaming
        case failed(String)
    }

    public struct Status: Equatable {
        public var state: State = .idle
        public var target = ""
        public var receiverName = ""
        public var streams: UInt32 = 0
        public var depthMode: UInt8 = 0
        public var roundTripMs: Double?
        public var framesSent = 0
        public var bytesSent = 0

        public init() {}
    }

    /// What this sender announces in HELLO.
    public struct Identity {
        public var name: String
        public var capabilities: UInt32
        public var maxTracked: UInt8
        public var sourceKind: UInt8

        public init(name: String, capabilities: UInt32, maxTracked: UInt8, sourceKind: UInt8) {
            self.name = name
            self.capabilities = capabilities
            self.maxTracked = maxTracked
            self.sourceKind = sourceKind
        }
    }

    /// Called on the sender's queue whenever the status changes (at most a few times per second).
    public var onStatus: ((Status) -> Void)?

    private let queue = DispatchQueue(label: "l2k.sender")
    private var connection: NWConnection?
    private var helloTimer: DispatchSourceTimer?
    private let session = UInt32.random(in: 1...UInt32.max)
    private var seq: UInt32 = 0
    private var identity = Identity(name: "Lidar2Kinect", capabilities: 0, maxTracked: 1, sourceKind: 0)
    private var status = Status()
    private var lastConfigTime: UInt64 = 0
    private var packet = [UInt8](repeating: 0, count: L2K.maxDatagram)
    private var depthEncoded = [UInt8](repeating: 0, count: L2K.depthWidth * L2K.depthHeight * 3)

    // What the receiver currently wants; read from the capture queue.
    private let lock = NSLock()
    private var wantedStreams: UInt32 = 0
    private var wantedDepthMode: UInt8 = 0
    private var wantedQuality: UInt8 = 0

    public init() {}

    /// Streams the receiver asked for (0 while not streaming), its depth mode, and its JPEG quality (0 = default).
    public var wanted: (streams: UInt32, depthMode: UInt8, quality: UInt8) {
        lock.lock()
        defer { lock.unlock() }
        return (wantedStreams, wantedDepthMode, wantedQuality)
    }

    public func setIdentity(_ identity: Identity) {
        queue.async { self.identity = identity }
    }

    public func connect(host: String, port: UInt16) {
        guard let nwPort = NWEndpoint.Port(rawValue: port) else { return }
        connect(to: .hostPort(host: NWEndpoint.Host(host), port: nwPort), label: "\(host):\(port)")
    }

    /// Connect to an endpoint found by `L2KDiscovery`.
    public func connect(to endpoint: NWEndpoint, label: String) {
        queue.async { [weak self] in
            guard let self else { return }
            self.teardown()
            let parameters = NWParameters.udp
            parameters.allowLocalEndpointReuse = true
            let connection = NWConnection(to: endpoint, using: parameters)
            self.connection = connection
            self.status = Status()
            self.status.state = .connecting
            self.status.target = label
            self.publish()
            connection.stateUpdateHandler = { [weak self, weak connection] state in
                guard let self, let connection, self.connection === connection else { return }
                switch state {
                case .ready:
                    self.status.state = .waitingForReceiver
                    self.publish()
                    self.receive(on: connection)
                    self.startHello()
                case .waiting(let error):
                    self.status.state = .failed(error.localizedDescription)
                    self.publish()
                case .failed(let error):
                    self.status.state = .failed(error.localizedDescription)
                    self.publish()
                    self.stopHello()
                default:
                    break
                }
            }
            connection.start(queue: self.queue)
        }
    }

    public func disconnect() {
        queue.async {
            if let connection = self.connection, self.status.state == .streaming || self.status.state == .waitingForReceiver {
                let n = l2k_encode_bye(&self.packet, L2K.maxDatagram, self.session, self.nextSeq())
                connection.send(content: Data(self.packet[0..<n]), completion: .idempotent)
            }
            self.teardown()
            self.status = Status()
            self.publish()
        }
    }

    /// Send one frame: the skeleton, and the images the receiver asked for when they are given.
    /// `depth` is 320x240 packed pixels, `jpeg` a 640x480 JPEG.
    public func send(skeleton: l2k_skeleton_frame, depth: [UInt16]?, jpeg: Data?) {
        transmit(skeleton: skeleton, withSkeleton: true, depth: depth, jpeg: jpeg)
    }

    /// Send a frame's skeleton alone, at once; its images follow with `sendImages` when they are made. The packets and
    /// their order are those of `send` - the skeleton only leaves earlier.
    public func send(skeleton: l2k_skeleton_frame) {
        transmit(skeleton: skeleton, withSkeleton: true, depth: nil, jpeg: nil)
    }

    /// Send the images of a frame whose skeleton went out with `send(skeleton:)`.
    public func sendImages(of skeleton: l2k_skeleton_frame, depth: [UInt16]?, jpeg: Data?) {
        guard depth != nil || jpeg != nil else { return }
        transmit(skeleton: skeleton, withSkeleton: false, depth: depth, jpeg: jpeg)
    }

    private func transmit(skeleton: l2k_skeleton_frame, withSkeleton: Bool, depth: [UInt16]?, jpeg: Data?) {
        queue.async {
            guard let connection = self.connection, self.status.state == .streaming else { return }
            var skeleton = skeleton
            let streams = self.status.streams
            var bytes = 0
            connection.batch {
                if withSkeleton, streams & L2K.streamSkeleton != 0 {
                    let n = l2k_encode_skeleton(&self.packet, L2K.maxDatagram, self.session, self.nextSeq(), &skeleton)
                    if n > 0 {
                        connection.send(content: Data(self.packet[0..<n]), completion: .idempotent)
                        bytes += n
                    }
                }
                if streams & L2K.streamDepth != 0, let depth, depth.count == L2K.depthWidth * L2K.depthHeight {
                    let length = l2k_depth_d1_encode(depth, depth.count, &self.depthEncoded, depth.count * 3)
                    if length > 0 {
                        bytes += self.depthEncoded.withUnsafeBufferPointer {
                            self.sendImage(UInt8(L2K_PKT_DEPTH), encoding: UInt8(L2K_DEPTH_D1), width: L2K.depthWidth, height: L2K.depthHeight,
                                           data: $0.baseAddress!, length: length, skeleton: skeleton, on: connection)
                        }
                    }
                }
                if streams & L2K.streamColor != 0, let jpeg, !jpeg.isEmpty {
                    bytes += jpeg.withUnsafeBytes {
                        self.sendImage(UInt8(L2K_PKT_COLOR), encoding: UInt8(L2K_COLOR_JPEG), width: L2K.colorWidth, height: L2K.colorHeight,
                                       data: $0.bindMemory(to: UInt8.self).baseAddress!, length: jpeg.count, skeleton: skeleton, on: connection)
                    }
                }
            }
            if withSkeleton { self.status.framesSent += 1 }
            self.status.bytesSent += bytes
        }
    }

    // MARK: - private (all on `queue`)

    private func nextSeq() -> UInt32 {
        seq &+= 1
        return seq
    }

    private func sendImage(_ type: UInt8, encoding: UInt8, width: Int, height: Int, data: UnsafePointer<UInt8>, length: Int,
                           skeleton: l2k_skeleton_frame, on connection: NWConnection) -> Int {
        var chunk = l2k_chunk()
        chunk.timestamp_us = skeleton.timestamp_us
        chunk.frame_number = skeleton.frame_number
        chunk.width = UInt16(width)
        chunk.height = UInt16(height)
        chunk.encoding = encoding
        var bytes = 0
        for index in 0..<l2k_chunk_count(length) {
            guard l2k_chunk_slice(&chunk, data, length, index) == 1 else { break }
            let n = l2k_encode_chunk(&packet, L2K.maxDatagram, session, nextSeq(), type, &chunk)
            guard n > 0 else { break }
            connection.send(content: Data(packet[0..<n]), completion: .idempotent)
            bytes += n
        }
        return bytes
    }

    private func startHello() {
        stopHello()
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(Int(L2K_HELLO_INTERVAL_MS)))
        timer.setEventHandler { [weak self] in self?.helloTick() }
        helloTimer = timer
        timer.resume()
    }

    private func stopHello() {
        helloTimer?.cancel()
        helloTimer = nil
    }

    private func helloTick() {
        guard let connection else { return }
        let now = L2K.nowMicroseconds()
        if status.state == .streaming, now - lastConfigTime > UInt64(L2K_PEER_TIMEOUT_MS) * 1000 {
            // the receiver went silent: stop streaming, keep calling
            status.state = .waitingForReceiver
            status.streams = 0
            status.roundTripMs = nil
            setWanted(streams: 0, depthMode: 0, quality: 0)
        }
        var hello = l2k_hello()
        hello.sender_time_us = now
        hello.caps = identity.capabilities
        hello.max_tracked = identity.maxTracked
        hello.source_kind = identity.sourceKind
        l2kSetName(&hello.name, identity.name)
        let n = l2k_encode_hello(&packet, L2K.maxDatagram, session, nextSeq(), &hello)
        connection.send(content: Data(packet[0..<n]), completion: .idempotent)
        publish()
    }

    private func receive(on connection: NWConnection) {
        connection.receiveMessage { [weak self, weak connection] data, _, _, error in
            guard let self, let connection, self.connection === connection else { return }
            if let data, !data.isEmpty { self.handle(data) }
            if error == nil { self.receive(on: connection) }
        }
    }

    private func handle(_ data: Data) {
        var header = l2k_header()
        var config = l2k_config()
        let result: (Int32, Int32) = data.withUnsafeBytes { raw in
            let bytes = raw.bindMemory(to: UInt8.self).baseAddress!
            return (l2k_decode_header(bytes, raw.count, &header), l2k_decode_config(bytes, raw.count, &config))
        }
        guard result.0 == L2K_OK else { return }
        if header.type == UInt8(L2K_PKT_BYE) {
            status.state = .waitingForReceiver
            status.streams = 0
            setWanted(streams: 0, depthMode: 0, quality: 0)
            publish()
            return
        }
        guard header.type == UInt8(L2K_PKT_CONFIG), result.1 == L2K_OK else { return }
        let now = L2K.nowMicroseconds()
        lastConfigTime = now
        let streams = config.streams & identity.capabilities
        let changed = status.state != .streaming || status.streams != streams || status.depthMode != config.depth_mode
        status.state = .streaming
        status.receiverName = l2kGetName(config.name)
        status.streams = streams
        status.depthMode = config.depth_mode
        if config.echo_sender_time_us != 0, now >= config.echo_sender_time_us {
            status.roundTripMs = Double(now - config.echo_sender_time_us) / 1000
        }
        setWanted(streams: status.streams, depthMode: config.depth_mode, quality: config.color_quality)
        if changed { publish() }
    }

    private func setWanted(streams: UInt32, depthMode: UInt8, quality: UInt8) {
        lock.lock()
        wantedStreams = streams
        wantedDepthMode = depthMode
        wantedQuality = quality
        lock.unlock()
    }

    private func teardown() {
        stopHello()
        connection?.stateUpdateHandler = nil
        connection?.cancel()
        connection = nil
        setWanted(streams: 0, depthMode: 0, quality: 0)
    }

    private func publish() {
        onStatus?(status)
    }
}
