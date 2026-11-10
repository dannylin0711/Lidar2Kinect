import CoreImage
import CoreImage.CIFilterBuiltins
import Vision
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

    /// 2D joints + a depth map give back the 3D joints they came from, in the camera space the C mapper expects.
    func testDepthLiftRecoversTheBody() throws {
        // a camera like the phone's wide camera, principal point slightly off-centre
        let camera = l2k_src_camera(fxn: 0.78, fyn: 1.04, cxn: 0.51, cyn: 0.49)
        var truth = l2k_src_body()
        l2k_synth_body(0.4, 0, 1, &truth)
        let positions = withUnsafeBytes(of: truth.p) { Array($0.bindMemory(to: l2k_vec3.self)) }
        let radius: Float = 0.05
        let sources = [L2K_SRC_PELVIS, L2K_SRC_NECK, L2K_SRC_HEAD, L2K_SRC_SHOULDER_LEFT, L2K_SRC_ELBOW_LEFT, L2K_SRC_WRIST_LEFT,
                       L2K_SRC_SHOULDER_RIGHT, L2K_SRC_ELBOW_RIGHT, L2K_SRC_WRIST_RIGHT, L2K_SRC_HIP_LEFT, L2K_SRC_KNEE_LEFT,
                       L2K_SRC_ANKLE_LEFT, L2K_SRC_HIP_RIGHT, L2K_SRC_KNEE_RIGHT, L2K_SRC_ANKLE_RIGHT].map { Int($0) }

        // Scene: a wall 5 m away; around each joint a patch of body surface `radius` in front of the joint, on the mask.
        let w = 256, h = 192
        var depth = [Float](repeating: 5.0, count: w * h)
        var mask = [UInt8](repeating: 0, count: w * h)
        var joints: [DepthLiftJoint] = []
        for source in sources {
            let p = positions[source]
            let forward = -p.z
            let u = camera.cxn + camera.fxn * p.x / forward, v = camera.cyn - camera.fyn * p.y / forward
            joints.append(DepthLiftJoint(source: source, u: u, v: v, radius: radius))
            let cx = Int(u * Float(w)), cy = Int(v * Float(h))
            for y in max(0, cy - 3)...min(h - 1, cy + 3) {
                for x in max(0, cx - 3)...min(w - 1, cx + 3) {
                    depth[y * w + x] = forward - radius
                    mask[y * w + x] = 255
                }
            }
        }
        // one more joint that falls on the wall: the right toe, off the body
        joints.append(DepthLiftJoint(source: Int(L2K_SRC_TOE_RIGHT), u: 0.05, v: 0.05, radius: radius))

        var body = l2k_src_body()
        let placed = depth.withUnsafeBytes { d in
            mask.withUnsafeBytes { m in
                DepthLift.lift(joints, buffers: DepthLiftBuffers(depth: d.baseAddress!, depthWidth: w, depthHeight: h, depthStride: w * 4,
                                                                 mask: m.baseAddress!, maskWidth: w, maskHeight: h, maskStride: w),
                               camera: camera, into: &body)
            }
        }
        XCTAssertEqual(placed, sources.count + 1)
        let lifted = withUnsafeBytes(of: body.p) { Array($0.bindMemory(to: l2k_vec3.self)) }
        let confidence = withUnsafeBytes(of: body.conf) { Array($0) }
        for source in sources {
            XCTAssertEqual(confidence[source], 2)
            XCTAssertEqual(lifted[source].x, positions[source].x, accuracy: 0.002)
            XCTAssertEqual(lifted[source].y, positions[source].y, accuracy: 0.002)
            XCTAssertEqual(lifted[source].z, positions[source].z, accuracy: 0.002)
        }
        // the joint on the wall is pulled to the body's depth and reported as estimated
        let toe = Int(L2K_SRC_TOE_RIGHT)
        XCTAssertEqual(confidence[toe], 1)
        XCTAssertEqual(lifted[toe].z, positions[Int(L2K_SRC_PELVIS)].z, accuracy: 0.25)

        // and the lifted body maps to the same Kinect skeleton as the original
        var mapperA = l2k_mapper(), mapperB = l2k_mapper()
        l2k_mapper_init(&mapperA, nil)
        l2k_mapper_init(&mapperB, nil)
        var frameA = l2k_skeleton_frame(), frameB = l2k_skeleton_frame()
        let up = l2k_vec3(x: 0, y: 1, z: 0)
        truth.tracking_id = 1
        body.tracking_id = 1
        l2k_mapper_map(&mapperA, &truth, 1, up, 0, 0, 0, &frameA)
        l2k_mapper_map(&mapperB, &body, 1, up, 0, 0, 0, &frameB)
        let a = frameA.slots[0], b = frameB.slots[0]
        XCTAssertTrue(a.isTracked && b.isTracked)
        for joint in [KinectJoint.head, .shoulderLeft, .wristRight, .kneeLeft, .ankleRight] {
            XCTAssertEqual(a.position(of: joint).x, b.position(of: joint).x, accuracy: 0.003)
            XCTAssertEqual(a.position(of: joint).y, b.position(of: joint).y, accuracy: 0.003)
            XCTAssertEqual(a.position(of: joint).z, b.position(of: joint).z, accuracy: 0.003)
        }
        XCTAssertFalse(b.jointStates.contains(UInt8(L2K_JOINT_NOT_TRACKED)))
    }

    /// Upper-body mode through the pipeline: half a body is tracked, complete, and flagged.
    func testUpperBodyMode() throws {
        let source = SyntheticSource(players: 1)
        var frame = source.frame(at: 0.5, timestamp: 10)
        // the camera sees nothing below the chest
        for joint in [L2K_SRC_PELVIS, L2K_SRC_HIP_LEFT, L2K_SRC_KNEE_LEFT, L2K_SRC_ANKLE_LEFT, L2K_SRC_TOE_LEFT,
                      L2K_SRC_HIP_RIGHT, L2K_SRC_KNEE_RIGHT, L2K_SRC_ANKLE_RIGHT, L2K_SRC_TOE_RIGHT] {
            withUnsafeMutableBytes(of: &frame.bodies[0].conf) { $0[Int(joint)] = 0 }
        }
        let normal = L2KPipeline().process(frame, kind: source.kind, wantDepth: false, wantColor: false, playersOnly: true)
        XCTAssertTrue(normal.skeleton.trackedSkeletons.isEmpty)

        let pipeline = L2KPipeline()
        pipeline.upperBodyMode = true
        let upper = pipeline.process(frame, kind: source.kind, wantDepth: true, wantColor: false, playersOnly: true)
        XCTAssertNotEqual(upper.skeleton.flags & UInt32(L2K_FRAME_UPPER_BODY), 0)
        let body = try XCTUnwrap(upper.skeleton.trackedSkeletons.first?.skeleton)
        XCTAssertFalse(body.jointStates.contains(UInt8(L2K_JOINT_NOT_TRACKED)))
        XCTAssertEqual(body.jointStates[KinectJoint.handLeft.rawValue], UInt8(L2K_JOINT_TRACKED))
        XCTAssertEqual(body.jointStates[KinectJoint.kneeLeft.rawValue], UInt8(L2K_JOINT_INFERRED))
        XCTAssertLessThan(body.position(of: .ankleLeft).y, body.position(of: .hipLeft).y)
        // the depth image still shows a whole figure, drawn from the completed skeleton
        XCTAssertGreaterThan(try XCTUnwrap(upper.depth).filter { l2k_depth_player($0) == 1 }.count, 1500)
    }

    /// Vision answers in the upright image; the source needs buffer coordinates. A QR code drawn at a known place in
    /// the buffer must come back at that place for every device roll (this pins the orientation <-> conversion pair).
    func testVisionPointsComeBackInBufferCoordinates() throws {
        let width = 640, height = 480
        let target = (u: Float(0.30), v: Float(0.70)) // buffer coordinates, origin top-left
        let generator = CIFilter.qrCodeGenerator()
        generator.message = Data("lidar2kinect".utf8)
        let code = try XCTUnwrap(generator.outputImage)
        let side: CGFloat = 120
        let scale = side / code.extent.width
        // Core Image's origin is bottom-left: buffer row v from the top is y = height * (1 - v)
        let centre = CGPoint(x: CGFloat(target.u) * CGFloat(width), y: CGFloat(height) * CGFloat(1 - target.v))
        let placed = code.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
            .transformed(by: CGAffineTransform(translationX: centre.x - side / 2, y: centre.y - side / 2))
        let canvas = placed.composited(over: CIImage(color: .white)).cropped(to: CGRect(x: 0, y: 0, width: width, height: height))
        var buffer: CVPixelBuffer?
        XCTAssertEqual(CVPixelBufferCreate(nil, width, height, kCVPixelFormatType_32BGRA,
                                           [kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary] as CFDictionary, &buffer), kCVReturnSuccess)
        let image = try XCTUnwrap(buffer)
        CIContext().render(canvas, to: image)

        for quadrant in Int32(0)..<4 {
            let request = VNDetectBarcodesRequest()
            let handler = VNImageRequestHandler(cvPixelBuffer: image, orientation: VisionOrientation.orientation(forQuadrant: quadrant), options: [:])
            try handler.perform([request])
            let box = try XCTUnwrap(request.results?.first?.boundingBox, "no code found for quadrant \(quadrant)")
            let point = VisionOrientation.bufferPoint(fromVision: CGPoint(x: box.midX, y: box.midY), quadrant: quadrant)
            XCTAssertEqual(point.u, target.u, accuracy: 0.02, "quadrant \(quadrant)")
            XCTAssertEqual(point.v, target.v, accuracy: 0.02, "quadrant \(quadrant)")
        }
    }

    /// The zoomed-out preview shows the Kinect's own image as its central part.
    func testZoomedColourPreview() throws {
        let source = SyntheticSource(players: 1)
        let pipeline = L2KPipeline()
        let frame = pipeline.process(source.frame(at: 0.2, timestamp: 1), kind: source.kind, wantDepth: false, wantColor: true,
                                     playersOnly: true, colorPreviewZoom: 2)
        let wide = try XCTUnwrap(frame.colorPreview)
        XCTAssertEqual(frame.colorPreviewZoom, 2)
        XCTAssertEqual(wide.extent, CGRect(x: 0, y: 0, width: 640, height: 480))
        var pixels = [UInt8](repeating: 0, count: 640 * 480 * 4)
        pipeline.ciContext.render(wide, toBitmap: &pixels, rowBytes: 640 * 4, bounds: wide.extent,
                                  format: .BGRA8, colorSpace: CGColorSpace(name: CGColorSpace.sRGB))
        func red(_ x: Int, _ y: Int) -> UInt8 { pixels[(y * 640 + x) * 4 + 2] }
        func green(_ x: Int, _ y: Int) -> UInt8 { pixels[(y * 640 + x) * 4 + 1] }
        // The source's red corner mark sits at the Kinect image's top-right corner: now the inner frame's (480, 120).
        XCTAssertGreaterThan(red(478, 122), 200)
        XCTAssertLessThan(green(478, 122), 60)
        // Outside the source image (this source is exactly as wide as the Kinect) there is nothing: black.
        XCTAssertEqual(red(100, 60), 0)
        XCTAssertEqual(red(600, 440), 0)
        // Inside, the picture.
        XCTAssertGreaterThan(red(320, 240), 10)
    }

    /// Runs Vision's body pose on a picture if one is given (L2K_TEST_IMAGE=/path/to.png) and prints what it finds.
    func testVisionOnAPicture() throws {
        guard let path = ProcessInfo.processInfo.environment["L2K_TEST_IMAGE"] else { throw XCTSkip("no L2K_TEST_IMAGE") }
        let image = try XCTUnwrap(CIImage(contentsOf: URL(fileURLWithPath: path)))
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(nil, Int(image.extent.width), Int(image.extent.height), kCVPixelFormatType_32BGRA,
                            [kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary] as CFDictionary, &buffer)
        let pixels = try XCTUnwrap(buffer)
        CIContext().render(image, to: pixels)
        let people = VisionBodyDetector().detect(in: pixels, quadrant: 0)
        print("L2K-VISION 2D people: \(people.count)")
        for person in people {
            print("L2K-VISION 2D " + person.joints.map { String(format: "%d:(%.2f,%.2f)", $0.source, $0.u, $0.v) }.joined(separator: " "))
        }
        let detector = VisionBody3DDetector()
        let joints = detector.detect(in: pixels, quadrant: 0)
        print("L2K-VISION 3D joints: \(joints.count) height \(detector.lastHeight) error \(detector.lastError ?? "none")")
        print("L2K-VISION 3D " + joints.map { String(format: "%d:(%.2f,%.2f,%.2fm)", $0.source, $0.u, $0.v, $0.depth) }.joined(separator: " "))
        let hybrid = VisionHybridDetector()
        let mixed = hybrid.detect(in: pixels, quadrant: 0)
        print("L2K-VISION hybrid joints: \(mixed.count), from the picture \(hybrid.lastSeen), from the model \(hybrid.lastModelOnly)")
        print("L2K-VISION hybrid " + mixed.map { String(format: "%d:(%.2f,%.2f,%.2fm,c%d)", $0.source, $0.u, $0.v, $0.depth, $0.confidence) }.joined(separator: " "))
    }

    func testPersonTrackerKeepsIdentities() {
        var tracker = VisionPersonTracker()
        func person(_ x: Float) -> VisionPerson { VisionPerson(joints: [], centre: SIMD2(x, 0.5), size: 0.2) }
        let first = tracker.assign([person(0.3), person(0.7)])
        XCTAssertEqual(Set(first).count, 2)
        // the detections arrive in the other order and have moved a little: same ids
        let second = tracker.assign([person(0.72), person(0.28)])
        XCTAssertEqual(second, [first[1], first[0]])
        // one leaves, one stays; then a newcomer far away gets a new id
        let third = tracker.assign([person(0.29)])
        XCTAssertEqual(third, [first[0]])
        let fourth = tracker.assign([person(0.30), person(0.9)])
        XCTAssertEqual(fourth[0], first[0])
        XCTAssertFalse(first.contains(fourth[1]))
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
