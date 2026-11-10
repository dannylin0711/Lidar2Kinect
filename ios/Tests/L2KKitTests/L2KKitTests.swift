import CoreImage
import L2KShared
import Network
import XCTest
@testable import L2KKit

final class L2KKitTests: XCTestCase {
    /// The Swift side reads the golden packets the C tests write (shared/tests/vectors).
    func testGoldenPacketsDecode() throws {
        let vectors = URL(fileURLWithPath: #filePath).deletingLastPathComponent()
            .appendingPathComponent("../../../shared/tests/vectors").standardizedFileURL
        let skeleton = try Data(contentsOf: vectors.appendingPathComponent("skeleton.bin"))
        var frame = l2k_skeleton_frame()
        let result = skeleton.withUnsafeBytes { l2k_decode_skeleton($0.bindMemory(to: UInt8.self).baseAddress, $0.count, &frame) }
        XCTAssertEqual(result, 0)
        XCTAssertEqual(frame.frame_number, 1234)
        XCTAssertEqual(frame.floor_plane.w, 1.25)
        let tracked = frame.trackedSkeletons
        XCTAssertEqual(tracked.count, 1)
        XCTAssertEqual(tracked[0].slot, 1)
        XCTAssertEqual(tracked[0].skeleton.tracking_id, 77)
        XCTAssertEqual(tracked[0].skeleton.position(of: .head).z, 2.0 + 3.0 / 128.0)
        XCTAssertEqual(frame.slots[4].state, UInt8(L2K_SKELETON_POSITION_ONLY))

        let hello = try Data(contentsOf: vectors.appendingPathComponent("hello.bin"))
        var decoded = l2k_hello()
        XCTAssertEqual(hello.withUnsafeBytes { l2k_decode_hello($0.bindMemory(to: UInt8.self).baseAddress, $0.count, &decoded) }, 0)
        XCTAssertEqual(l2kGetName(decoded.name), "Test iPhone")
        var again = l2k_hello()
        again.sender_time_us = decoded.sender_time_us
        again.caps = decoded.caps
        again.max_tracked = decoded.max_tracked
        again.source_kind = decoded.source_kind
        l2kSetName(&again.name, "Test iPhone")
        var packet = [UInt8](repeating: 0, count: 128)
        let n = l2k_encode_hello(&packet, 128, 0xAABB_CCDD, 7, &again)
        XCTAssertEqual(Data(packet[0..<n]), hello)
    }

    func testPipelineWithSyntheticSource() throws {
        let source = SyntheticSource(players: 2)
        let pipeline = L2KPipeline()
        var frame: L2KFrame!
        for i in 0..<10 {
            let t = Double(i) / 30
            frame = pipeline.process(source.frame(at: t, timestamp: 100 + t), kind: source.kind,
                                     wantDepth: true, wantColor: true, playersOnly: true)
        }
        XCTAssertEqual(frame.skeleton.frame_number, 10)
        XCTAssertEqual(frame.skeleton.trackedSkeletons.map(\.slot), [0, 1])
        XCTAssertEqual(frame.skeleton.timestamp_us, 100_300_000)
        XCTAssertEqual(frame.skeleton.elevationDegrees, 0, accuracy: 0.01)
        XCTAssertEqual(frame.quadrant, 0)

        let depth = try XCTUnwrap(frame.depth)
        XCTAssertEqual(depth.count, 320 * 240)
        XCTAssertGreaterThan(depth.filter { l2k_depth_player($0) == 1 }.count, 1500)
        XCTAssertGreaterThan(depth.filter { l2k_depth_player($0) == 2 }.count, 1500)

        // Colour: 640x480, mirrored. The source marks its top-left corner red, so the red mark is top-RIGHT here.
        let color = try XCTUnwrap(frame.color)
        XCTAssertEqual(color.extent, CGRect(x: 0, y: 0, width: 640, height: 480))
        var pixels = [UInt8](repeating: 0, count: 640 * 480 * 4)
        pipeline.ciContext.render(color, toBitmap: &pixels, rowBytes: 640 * 4, bounds: color.extent,
                                  format: .BGRA8, colorSpace: CGColorSpace(name: CGColorSpace.sRGB))
        func pixel(_ x: Int, _ y: Int) -> (b: UInt8, g: UInt8, r: UInt8) {
            let i = (y * 640 + x) * 4
            return (pixels[i], pixels[i + 1], pixels[i + 2])
        }
        XCTAssertGreaterThan(pixel(636, 3).r, 200)
        XCTAssertLessThan(pixel(636, 3).g, 60)
        XCTAssertLessThan(pixel(3, 3).r, 120)
        XCTAssertLessThan(pixel(636, 476).r, 120)

        let jpeg = try XCTUnwrap(pipeline.jpeg(color, quality: 0.6))
        XCTAssertEqual(jpeg.prefix(2), Data([0xFF, 0xD8]))
        XCTAssertLessThan(jpeg.count, 200_000)
    }

    /// The sender against a minimal receiver on localhost: HELLO -> CONFIG -> the requested streams arrive and decode.
    func testSenderHandshakeAndStreams() throws {
        let listener = try NWListener(using: .udp, on: .any)
        let queue = DispatchQueue(label: "test.receiver")
        let gotSkeleton = expectation(description: "skeleton")
        let gotDepth = expectation(description: "depth")
        let gotColor = expectation(description: "colour")
        var seen = Set<UInt8>()
        var storage = [UInt8](repeating: 0, count: 320 * 240 * 3)
        var colorStorage = [UInt8](repeating: 0, count: 1_228_800)
        var depthAssembler = l2k_assembler()
        var colorAssembler = l2k_assembler()
        storage.withUnsafeMutableBufferPointer { l2k_assembler_init(&depthAssembler, $0.baseAddress, $0.count) }
        colorStorage.withUnsafeMutableBufferPointer { l2k_assembler_init(&colorAssembler, $0.baseAddress, $0.count) }
        var receiverSeq: UInt32 = 0

        func handle(_ data: Data, on connection: NWConnection) {
            data.withUnsafeBytes { raw in
                let bytes = raw.bindMemory(to: UInt8.self).baseAddress!
                var header = l2k_header()
                guard l2k_decode_header(bytes, raw.count, &header) == L2K_OK else { return XCTFail("bad header") }
                switch Int32(header.type) {
                case Int32(L2K_PKT_HELLO):
                    var hello = l2k_hello()
                    XCTAssertEqual(l2k_decode_hello(bytes, raw.count, &hello), 0)
                    XCTAssertEqual(l2kGetName(hello.name), "unit test")
                    var config = l2k_config()
                    config.echo_sender_time_us = hello.sender_time_us
                    config.streams = hello.caps
                    config.consumer_kind = UInt8(L2K_CONSUMER_MONITOR)
                    l2kSetName(&config.name, "test receiver")
                    var packet = [UInt8](repeating: 0, count: 128)
                    receiverSeq += 1
                    let n = l2k_encode_config(&packet, 128, 99, receiverSeq, &config)
                    connection.send(content: Data(packet[0..<n]), completion: .idempotent)
                case Int32(L2K_PKT_SKELETON):
                    var frame = l2k_skeleton_frame()
                    XCTAssertEqual(l2k_decode_skeleton(bytes, raw.count, &frame), 0)
                    if frame.trackedSkeletons.count == 1, seen.insert(header.type).inserted { gotSkeleton.fulfill() }
                case Int32(L2K_PKT_DEPTH), Int32(L2K_PKT_COLOR):
                    var chunk = l2k_chunk()
                    XCTAssertEqual(l2k_decode_chunk(bytes, raw.count, &chunk), 0)
                    let isDepth = Int32(header.type) == Int32(L2K_PKT_DEPTH)
                    let done = isDepth ? l2k_assembler_push(&depthAssembler, &chunk) : l2k_assembler_push(&colorAssembler, &chunk)
                    guard done == 1, seen.insert(header.type).inserted else { return }
                    if isDepth {
                        var pixels = [UInt16](repeating: 0, count: 320 * 240)
                        XCTAssertEqual(l2k_depth_d1_decode(depthAssembler.data, Int(depthAssembler.info.total_len), &pixels, pixels.count), 0)
                        XCTAssertGreaterThan(pixels.filter { l2k_depth_player($0) == 1 }.count, 1500)
                        gotDepth.fulfill()
                    } else {
                        XCTAssertEqual(colorAssembler.data[0], 0xFF)
                        XCTAssertEqual(colorAssembler.data[1], 0xD8)
                        gotColor.fulfill()
                    }
                default:
                    break
                }
            }
        }
        func receive(on connection: NWConnection) {
            connection.receiveMessage { data, _, _, error in
                if let data, !data.isEmpty { handle(data, on: connection) }
                if error == nil { receive(on: connection) }
            }
        }
        listener.newConnectionHandler = { connection in
            connection.start(queue: queue)
            receive(on: connection)
        }
        let ready = expectation(description: "listener ready")
        listener.stateUpdateHandler = { if case .ready = $0 { ready.fulfill() } }
        listener.start(queue: queue)
        wait(for: [ready], timeout: 5)
        let port = try XCTUnwrap(listener.port?.rawValue)

        let session = L2KSession()
        session.deviceName = "unit test"
        let streaming = expectation(description: "streaming")
        streaming.assertForOverFulfill = false
        session.sender.onStatus = { status in
            if status.state == .streaming {
                XCTAssertEqual(status.receiverName, "test receiver")
                streaming.fulfill()
            }
        }
        try session.start(SyntheticSource(players: 1))
        session.sender.connect(host: "127.0.0.1", port: port)
        wait(for: [streaming, gotSkeleton, gotDepth, gotColor], timeout: 10)
        session.sender.disconnect()
        session.stop()
        listener.cancel()
    }
}
