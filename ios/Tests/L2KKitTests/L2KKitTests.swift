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

    /// The capture format with the whole view: the 4:3 one that reads the whole sensor, not a 16:9 crop of it, and
    /// not a bigger frame than body detection needs.
    func testWidestFormatIsTheWholeSensor() {
        let formats = [
            L2KCameraFormat(width: 1280, height: 720, fieldOfView: 105, maxFrameRate: 60),
            L2KCameraFormat(width: 1920, height: 1080, fieldOfView: 105, maxFrameRate: 60),
            L2KCameraFormat(width: 640, height: 480, fieldOfView: 105, maxFrameRate: 30),
            L2KCameraFormat(width: 1920, height: 1440, fieldOfView: 105, maxFrameRate: 30),
            L2KCameraFormat(width: 1440, height: 1080, fieldOfView: 105, maxFrameRate: 30),
            L2KCameraFormat(width: 4032, height: 3024, fieldOfView: 105, maxFrameRate: 30),
            L2KCameraFormat(width: 4032, height: 3024, fieldOfView: 106, maxFrameRate: 15), // too slow for 30 frames/s
            L2KCameraFormat(width: 3840, height: 2160, fieldOfView: 96, maxFrameRate: 60),  // a cropped 4K format
        ]
        XCTAssertEqual(L2KCameraView.widestFormat(formats), 4)
        XCTAssertEqual(formats[4].verticalFieldOfView, 88.7, accuracy: 0.2)
        XCTAssertEqual(formats[0].verticalFieldOfView, 72.5, accuracy: 0.2) // the old 1280x720 choice
        // Nothing wide is big enough: the largest of the widest.
        XCTAssertEqual(L2KCameraView.widestFormat([formats[2], L2KCameraFormat(width: 1024, height: 768, fieldOfView: 105, maxFrameRate: 30)]), 1)
        // The whole sensor only comes photo-sized (4032 wide): a smaller frame with less view is taken instead ...
        XCTAssertEqual(L2KCameraView.widestFormat([formats[0], formats[1], formats[5]]), 0)
        // ... unless there is nothing smaller.
        XCTAssertEqual(L2KCameraView.widestFormat([formats[5]]), 0)
        // Nothing reaches 30 frames/s: still an answer.
        XCTAssertEqual(L2KCameraView.widestFormat([formats[6]]), 0)
        XCTAssertNil(L2KCameraView.widestFormat([]))
    }

    /// A 60 fps camera: the widest view among the formats that run that fast, and every second frame is detected on.
    func testFastCameraFormatAndFrames() {
        // the iPhone 17 Pro's front camera (read on the device, 2026-11-11)
        let front = [
            L2KCameraFormat(width: 640, height: 480, fieldOfView: 84.1, maxFrameRate: 60),
            L2KCameraFormat(width: 960, height: 540, fieldOfView: 84.1, maxFrameRate: 30),
            L2KCameraFormat(width: 1024, height: 768, fieldOfView: 84.1, maxFrameRate: 60),
            L2KCameraFormat(width: 1280, height: 720, fieldOfView: 84.1, maxFrameRate: 30),
            L2KCameraFormat(width: 1280, height: 1280, fieldOfView: 84.1, maxFrameRate: 30),
        ]
        XCTAssertEqual(L2KCameraView.widestFormat(front), 4)
        XCTAssertEqual(L2KCameraView.widestFormat(front, frameRate: 60), 2)
        XCTAssertEqual(front[2].verticalFieldOfView, 68.1, accuracy: 0.2)
        XCTAssertFalse(L2KCameraView.fastFormatKeepsView(front)) // 84° × 68° instead of 84° × 84°
        XCTAssertFalse(L2KCameraView.fastFormatKeepsView([front[1], front[3], front[4]])) // nothing runs at 60
        // the iPad Pro 11" (M2) front camera (read on the device, 2026-11-22): the whole view at both rates
        let pad = [
            L2KCameraFormat(width: 1280, height: 720, fieldOfView: 106.8, maxFrameRate: 60),
            L2KCameraFormat(width: 1440, height: 1080, fieldOfView: 101.4, maxFrameRate: 60),
            L2KCameraFormat(width: 1920, height: 1440, fieldOfView: 106.8, maxFrameRate: 30),
            L2KCameraFormat(width: 1920, height: 1440, fieldOfView: 106.8, maxFrameRate: 60),
        ]
        XCTAssertEqual(L2KCameraView.widestFormat(pad, frameRate: 60), 3)
        XCTAssertTrue(L2KCameraView.fastFormatKeepsView(pad))
        // no format runs at 60: the 30 fps choice
        XCTAssertEqual(L2KCameraView.widestFormat([front[1], front[3], front[4]], frameRate: 60), 2)

        func usedOf(_ times: [Double]) -> [Int] {
            var last: Double?, used: [Int] = []
            for (index, time) in times.enumerated() where L2KCameraView.usesFrame(at: time, last: last) {
                last = time
                used.append(index)
            }
            return used
        }
        XCTAssertEqual(usedOf((0..<8).map { Double($0) / 60 }), [0, 2, 4, 6])
        XCTAssertEqual(usedOf((0..<5).map { Double($0) / 30 }), [0, 1, 2, 3, 4])
        // frames a few milliseconds early or late at 30 fps are all used
        XCTAssertEqual(usedOf([0, 0.030, 0.068, 0.099, 0.135]), [0, 1, 2, 3, 4])
        // a 60 fps camera that lost the frame it would have used: the next one, not the one after
        XCTAssertEqual(usedOf([0, 1.0 / 60, 3.0 / 60, 4.0 / 60, 5.0 / 60]), [0, 2, 4])
        // the camera's clock starts again
        XCTAssertEqual(usedOf([10, 0, 1.0 / 30]), [0, 1, 2])
    }

    /// The skeleton is handed on before the images: `onSkeleton` comes first, with no image and the same skeleton,
    /// and a person mask that costs time is only made when an image wants it.
    func testSkeletonBeforeImages() throws {
        let session = L2KSession()
        session.localWantsDepth = true
        session.localRate = 30
        let lock = NSLock()
        var order: [String] = []
        var early: [UInt32: l2k_skeleton_frame] = [:]
        var mismatches = 0, earlyImages = 0, depthFrames = 0
        let enough = expectation(description: "frames")
        enough.assertForOverFulfill = false
        session.onSkeleton = { frame in
            lock.lock()
            order.append("s\(frame.skeleton.frame_number)")
            early[frame.skeleton.frame_number] = frame.skeleton
            if frame.depth != nil || frame.color != nil { earlyImages += 1 }
            lock.unlock()
        }
        session.onFrame = { frame in
            lock.lock()
            order.append("f\(frame.skeleton.frame_number)")
            var a = early[frame.skeleton.frame_number] ?? l2k_skeleton_frame(), b = frame.skeleton
            if memcmp(&a, &b, MemoryLayout<l2k_skeleton_frame>.size) != 0 { mismatches += 1 }
            if frame.depth != nil { depthFrames += 1 }
            let done = order.count >= 12
            lock.unlock()
            if done { enough.fulfill() }
        }
        try session.start(SyntheticSource(players: 1))
        wait(for: [enough], timeout: 10)
        session.stop()
        lock.lock()
        defer { lock.unlock() }
        XCTAssertEqual(mismatches, 0)
        XCTAssertEqual(earlyImages, 0)
        XCTAssertGreaterThan(depthFrames, 3)
        for pair in stride(from: 0, to: order.count - 1, by: 2) {
            XCTAssertEqual(order[pair].first, "s")
            XCTAssertEqual(order[pair + 1], "f" + order[pair].dropFirst())
        }

        // The mask on demand: not made for a frame without an image, made once for a frame with one.
        let pipeline = L2KPipeline()
        var made = 0
        var mask: CVPixelBuffer?
        CVPixelBufferCreate(nil, 64, 48, kCVPixelFormatType_OneComponent8, [kCVPixelBufferIOSurfacePropertiesKey: [:]] as CFDictionary, &mask)
        var source = L2KSourceFrame(timestamp: 0, bodies: [], upCamera: SIMD3(0, 1, 0),
                                    camera: l2k_src_camera(fxn: 1, fyn: 1, cxn: 0.5, cyn: 0.5))
        source.makeSegmentation = { made += 1; return mask }
        var frame = pipeline.skeletonFrame(source, kind: UInt8(L2K_SOURCE_VISION_3D))
        XCTAssertEqual(made, 0)
        pipeline.addImages(to: &frame, from: source, wantDepth: false, wantColor: false, playersOnly: false)
        XCTAssertEqual(made, 0)
        XCTAssertNil(frame.depth)
        pipeline.addImages(to: &frame, from: source, wantDepth: true, wantColor: false, playersOnly: false)
        XCTAssertEqual(made, 1)
        XCTAssertNotNil(frame.depth)
        // `process` is the two halves
        let whole = pipeline.process(source, kind: UInt8(L2K_SOURCE_VISION_3D), wantDepth: true, wantColor: false, playersOnly: false)
        XCTAssertEqual(made, 2)
        XCTAssertEqual(whole.depth, frame.depth)
    }

    /// The whole-view preview: just wide enough for the camera image, for every roll of the device.
    func testPreviewZoomShowsTheWholeCamera() throws {
        // A 4:3 camera 105 degrees wide, centred.
        let tanH = tan(Float(105) * .pi / 360), tanV = tanH * 3 / 4
        let camera = l2k_src_camera(fxn: 0.5 / tanH, fyn: 0.5 / tanV, cxn: 0.5, cyn: 0.5)
        var cam = camera
        for quadrant in Int32(0)..<4 {
            let zoom = L2KCameraView.zoomToFit(camera: camera, quadrant: quadrant)
            // Every corner of the camera image is inside the zoomed view, and one of them is on its edge.
            var reach: Float = 0
            let lo = 0.5 - zoom / 2, hi = 0.5 + zoom / 2
            var ax: Float = 0, ay: Float = 0, bx: Float = 0, by: Float = 0
            l2k_virtual_to_source(&cam, quadrant, lo, lo, &ax, &ay)
            l2k_virtual_to_source(&cam, quadrant, hi, hi, &bx, &by)
            let sLo = SIMD2(ax, ay), sHi = SIMD2(bx, by)
            let low = simd_min(sLo, sHi), high = simd_max(sLo, sHi)
            XCTAssertLessThanOrEqual(low.x, 1e-4, "quadrant \(quadrant)")
            XCTAssertLessThanOrEqual(low.y, 1e-4, "quadrant \(quadrant)")
            XCTAssertGreaterThanOrEqual(high.x, 1 - 1e-4, "quadrant \(quadrant)")
            XCTAssertGreaterThanOrEqual(high.y, 1 - 1e-4, "quadrant \(quadrant)")
            reach = min(abs(low.x), abs(low.y), abs(high.x - 1), abs(high.y - 1))
            XCTAssertLessThan(reach, 1e-3, "quadrant \(quadrant): the view is wider than it needs to be")
        }
        // Landscape: the 4:3 camera and the 4:3 Kinect image have the same shape, so the zoom is the ratio of the views.
        XCTAssertEqual(L2KCameraView.zoomToFit(camera: camera, quadrant: 0), 2.33, accuracy: 0.05)
        // The synthetic source is exactly as wide as the Kinect: nothing to zoom out for.
        let source = SyntheticSource(players: 1)
        XCTAssertEqual(L2KCameraView.zoomToFit(camera: source.frame(at: 0, timestamp: 1).camera, quadrant: 0), 1, accuracy: 0.01)
        // And through the pipeline: the fit replaces the fixed zoom.
        let pipeline = L2KPipeline()
        var frame = source.frame(at: 0.2, timestamp: 1)
        frame.camera = camera
        let out = pipeline.process(frame, kind: source.kind, wantDepth: false, wantColor: true, playersOnly: true, colorPreviewFit: true)
        XCTAssertNotNil(out.colorPreview)
        XCTAssertEqual(out.colorPreviewZoom, 2.33, accuracy: 0.05)
    }

    /// The system's level angle as a roll quadrant, and a camera that is mounted another way round than assumed.
    func testCameraMountingFromTheLevelAngle() {
        XCTAssertEqual(L2KCameraView.quadrant(forLevelAngle: 0), 0)
        XCTAssertEqual(L2KCameraView.quadrant(forLevelAngle: 90), 3)  // an iPhone in portrait: the buffer's left is up
        XCTAssertEqual(L2KCameraView.quadrant(forLevelAngle: 180), 2)
        XCTAssertEqual(L2KCameraView.quadrant(forLevelAngle: 270), 1)
        XCTAssertEqual(L2KCameraView.quadrant(forLevelAngle: 360), 0)
        XCTAssertEqual(l2k_quadrant_from_up(-1, 0), 3)

        // Mounted as assumed: nothing changes.
        var mounting = L2KCameraMounting()
        let portrait = simd_normalize(SIMD3<Float>(-0.95, 0.05, 0.3))
        for _ in 0..<40 { XCTAssertEqual(mounting.correct(up: portrait, levelAngle: 90), portrait) }
        XCTAssertEqual(mounting.quarterTurns, 0)
        XCTAssertEqual(mounting.agreed, 40)

        // The system says the frames are level as they are, while gravity under the assumed mounting says "left is
        // up": the sensor sits a quarter turn round. Believed after a steady run, not after one frame.
        mounting = L2KCameraMounting()
        XCTAssertEqual(mounting.correct(up: portrait, levelAngle: 0), portrait)
        var up = portrait
        for _ in 0..<20 { up = mounting.correct(up: portrait, levelAngle: 0) }
        XCTAssertNotEqual(mounting.quarterTurns, 0)
        XCTAssertEqual(l2k_quadrant_from_up(up.x, up.y), 0)
        XCTAssertEqual(up.z, portrait.z) // the tilt towards the camera is not touched

        // A device lying flat, or held on a diagonal, says nothing.
        mounting = L2KCameraMounting()
        for _ in 0..<40 {
            _ = mounting.correct(up: SIMD3(0.05, 0.02, 0.99), levelAngle: 180)
            _ = mounting.correct(up: simd_normalize(SIMD3(0.7, 0.72, 0.1)), levelAngle: 180)
        }
        XCTAssertEqual(mounting.quarterTurns, 0)
        XCTAssertEqual(mounting.compared, 0)

        // What is left to turn when the connection has turned the frames already: the iPhone 17 Pro's front camera in
        // portrait (level angle 0°, frames turned 270°) needs the same quarter turn as a rear camera (90°, 0°).
        XCTAssertEqual(L2KCameraView.remainingTurn(levelAngle: 0, applied: 270), 90)
        XCTAssertEqual(L2KCameraView.remainingTurn(levelAngle: 90, applied: 0), 90)
        XCTAssertEqual(L2KCameraView.remainingTurn(levelAngle: 90, applied: 90), 0)
        XCTAssertEqual(L2KCameraView.remainingTurn(levelAngle: 270, applied: 90), 180)
        XCTAssertEqual(L2KCameraView.quadrant(forLevelAngle: L2KCameraView.remainingTurn(levelAngle: 0, applied: 270)), 3)

        // A camera whose mounting is known only counts.
        mounting = L2KCameraMounting(adopts: false)
        for _ in 0..<40 { _ = mounting.correct(up: portrait, levelAngle: 0) }
        XCTAssertEqual(mounting.quarterTurns, 0)
        XCTAssertEqual(mounting.agreed, 0)
        XCTAssertEqual(mounting.compared, 40)
    }

    /// The game's images are framed on the skeleton that is sent.
    func testGameViewFollowsTheSentSkeleton() throws {
        // The fit: picture points that are the sent points 1.6 times as spread around another centre.
        let sent: [SIMD2<Float>] = [SIMD2(160, 40), SIMD2(140, 90), SIMD2(182, 92), SIMD2(150, 200), SIMD2(171, 198), SIMD2(120, 120)]
        let sentCentre = sent.reduce(SIMD2<Float>(), +) / Float(sent.count)
        let pictureCentre = SIMD2<Float>(190, 150)
        let picture = sent.map { pictureCentre + 1.6 * ($0 - sentCentre) }
        let view = try XCTUnwrap(L2KGameView.fitting(sent: sent, picture: picture))
        XCTAssertEqual(view.scale, 1.6, accuracy: 1e-4)
        // every sent pixel p must show the picture pixel centre + scale * (p - centre) + shift
        for (p, q) in zip(sent, picture) {
            XCTAssertEqual(160 + view.scale * (p.x - 160) + view.dx * 320, q.x, accuracy: 0.01)
            XCTAssertEqual(120 + view.scale * (p.y - 120) + view.dy * 240, q.y, accuracy: 0.01)
        }
        // The same points: the Kinect's own view. Too few points, or points in one place: no view.
        XCTAssertTrue(try XCTUnwrap(L2KGameView.fitting(sent: sent, picture: sent)).isIdentity)
        XCTAssertNil(L2KGameView.fitting(sent: Array(sent.prefix(3)), picture: Array(picture.prefix(3))))
        XCTAssertNil(L2KGameView.fitting(sent: [SIMD2<Float>](repeating: SIMD2(10, 10), count: 6), picture: picture))

        // The camera that renders the view: the same source position as the plain camera at the scaled, shifted pixel.
        var camera = l2k_src_camera(fxn: 0.42, fyn: 0.63, cxn: 0.51, cyn: 0.48)
        for quadrant in 0..<4 {
            var moved = view.camera(camera, quadrant: Int32(quadrant))
            for (un, vn) in [(Float(0), Float(0)), (1, 0), (0.3, 0.8), (0.5, 0.5), (1, 1)] {
                var a: Float = 0, b: Float = 0, c: Float = 0, d: Float = 0
                l2k_virtual_to_source(&moved, Int32(quadrant), un, vn, &a, &b)
                l2k_virtual_to_source(&camera, Int32(quadrant), 0.5 + view.scale * (un - 0.5) + view.dx, 0.5 + view.scale * (vn - 0.5) + view.dy, &c, &d)
                XCTAssertEqual(a, c, accuracy: 1e-5, "quadrant \(quadrant)")
                XCTAssertEqual(b, d, accuracy: 1e-5, "quadrant \(quadrant)")
            }
        }

        // Through the pipeline: a player 1.2 m in front of a level camera, normalised to the game's 2.5 m. The images
        // have to show more than the Kinect's own view (the player is drawn smaller), and the player mask has to lie
        // on the skeleton that is sent.
        let pipeline = L2KPipeline()
        pipeline.normalize = true
        var body = l2k_src_body()
        body.tracking_id = 1
        let joints: [(Int, SIMD3<Float>)] = [
            (Int(L2K_SRC_PELVIS), SIMD3(0, -0.1, -1.2)), (Int(L2K_SRC_NECK), SIMD3(0, 0.4, -1.2)), (Int(L2K_SRC_HEAD), SIMD3(0, 0.6, -1.2)),
            (Int(L2K_SRC_SHOULDER_LEFT), SIMD3(-0.2, 0.4, -1.2)), (Int(L2K_SRC_ELBOW_LEFT), SIMD3(-0.3, 0.15, -1.2)), (Int(L2K_SRC_WRIST_LEFT), SIMD3(-0.32, -0.1, -1.2)),
            (Int(L2K_SRC_SHOULDER_RIGHT), SIMD3(0.2, 0.4, -1.2)), (Int(L2K_SRC_ELBOW_RIGHT), SIMD3(0.3, 0.15, -1.2)), (Int(L2K_SRC_WRIST_RIGHT), SIMD3(0.32, -0.1, -1.2)),
            (Int(L2K_SRC_HIP_LEFT), SIMD3(-0.1, -0.15, -1.2)), (Int(L2K_SRC_KNEE_LEFT), SIMD3(-0.1, -0.55, -1.2)), (Int(L2K_SRC_ANKLE_LEFT), SIMD3(-0.1, -0.95, -1.2)),
            (Int(L2K_SRC_HIP_RIGHT), SIMD3(0.1, -0.15, -1.2)), (Int(L2K_SRC_KNEE_RIGHT), SIMD3(0.1, -0.55, -1.2)), (Int(L2K_SRC_ANKLE_RIGHT), SIMD3(0.1, -0.95, -1.2)),
        ]
        for (joint, position) in joints { body.set(joint, position, confidence: 2) }
        // a wide camera, and a mask that is the player's bounding box in its picture
        let source = l2k_src_camera(fxn: 0.42, fyn: 0.56, cxn: 0.5, cyn: 0.5)
        let maskWidth = 200, maskHeight = 150
        var made: CVPixelBuffer?
        CVPixelBufferCreate(nil, maskWidth, maskHeight, kCVPixelFormatType_OneComponent8, nil, &made)
        let mask = try XCTUnwrap(made)
        CVPixelBufferLockBaseAddress(mask, [])
        let base = try XCTUnwrap(CVPixelBufferGetBaseAddress(mask)).assumingMemoryBound(to: UInt8.self)
        let stride = CVPixelBufferGetBytesPerRow(mask)
        func column(_ x: Float) -> Int { Int((source.cxn + source.fxn * x / 1.2) * Float(maskWidth)) }
        func row(_ y: Float) -> Int { Int((source.cyn - source.fyn * y / 1.2) * Float(maskHeight)) }
        for y in 0..<maskHeight {
            for x in 0..<maskWidth {
                base[y * stride + x] = (column(-0.4)...column(0.4)).contains(x) && (row(0.75)...row(-1.05)).contains(y) ? 255 : 0
            }
        }
        CVPixelBufferUnlockBaseAddress(mask, [])
        var frame = L2KFrame(skeleton: l2k_skeleton_frame(), depth: nil, color: nil, quadrant: 0)
        for step in 0..<20 { // the view is eased in
            frame = pipeline.process(L2KSourceFrame(timestamp: Double(step) / 30, bodies: [body], upCamera: SIMD3(0, 1, 0), camera: source,
                                                    depthMap: nil, segmentation: mask, image: nil),
                                     kind: UInt8(L2K_SOURCE_VISION_3D), wantDepth: true, wantColor: false, playersOnly: false)
        }
        let player = try XCTUnwrap(frame.skeleton.trackedSkeletons.first?.skeleton)
        print(String(format: "L2K-GAMEVIEW player at %.2f m, view scale %.3f shift %.3f %.3f", player.position.z, frame.gameView.scale, frame.gameView.dx, frame.gameView.dy))
        XCTAssertEqual(player.position.z, 2.5, accuracy: 0.05)
        XCTAssertEqual(frame.gameView.scale, 2.5 / 1.2, accuracy: 0.12) // the player is reported about twice as far away
        let depth = try XCTUnwrap(frame.depth)
        // joints of the sent skeleton are on player pixels; a point beside the body is not
        for joint in [KinectJoint.head, .hipCenter, .handLeft, .kneeRight] {
            var u: Float = 0, v: Float = 0
            XCTAssertEqual(l2k_skeleton_to_depth_pixel(player.position(of: joint), 320, 240, &u, &v), 1)
            XCTAssertNotEqual(depth[min(239, max(0, Int(v))) * 320 + min(319, max(0, Int(u)))] & UInt16(L2K_PLAYER_INDEX_MASK), 0, "\(joint)")
        }
        XCTAssertEqual(depth[120 * 320 + 20] & UInt16(L2K_PLAYER_INDEX_MASK), 0)

        // Switched off: the images are the middle of the camera picture again.
        pipeline.imagesFollowSkeleton = false
        frame = pipeline.process(L2KSourceFrame(timestamp: 1, bodies: [body], upCamera: SIMD3(0, 1, 0), camera: source,
                                                depthMap: nil, segmentation: mask, image: nil),
                                 kind: UInt8(L2K_SOURCE_VISION_3D), wantDepth: true, wantColor: false, playersOnly: false)
        XCTAssertTrue(frame.gameView.isIdentity)
    }

    /// The picture for Vision: its two fixed sizes, and where the camera image sits in it.
    func testVisionCanvasLayout() {
        // 4:3 camera, device upright in landscape: a wide picture, the image as high as the picture.
        var layout = VisionCanvasLayout(bufferWidth: 1920, bufferHeight: 1440, quadrant: 0)
        XCTAssertEqual([layout.width, layout.height], [1280, 720])
        XCTAssertEqual(layout.content, CGRect(x: 160, y: 0, width: 960, height: 720))
        // The same camera in a portrait device: a tall picture, bars above and below.
        layout = VisionCanvasLayout(bufferWidth: 1920, bufferHeight: 1440, quadrant: 3)
        XCTAssertEqual([layout.width, layout.height], [720, 1280])
        XCTAssertEqual(layout.content, CGRect(x: 0, y: 160, width: 720, height: 960))
        // A square camera (the iPhone 17 Pro's front camera) never gives Vision a square picture.
        for quadrant in 0..<4 {
            layout = VisionCanvasLayout(bufferWidth: 1280, bufferHeight: 1280, quadrant: Int32(quadrant))
            XCTAssertEqual([layout.width, layout.height], [720, 1280])
            XCTAssertEqual(layout.content, CGRect(x: 0, y: 280, width: 720, height: 720))
        }
        // The 16:9 camera of before 0.9: the picture is the whole image, as Vision saw it then.
        layout = VisionCanvasLayout(bufferWidth: 1280, bufferHeight: 720, quadrant: 3)
        XCTAssertEqual(layout.content, CGRect(x: 0, y: 0, width: 720, height: 1280))

        // Picture point <-> buffer point, and the intrinsics: the principal point and the focal length in picture pixels.
        for quadrant in 0..<4 {
            layout = VisionCanvasLayout(bufferWidth: 1920, bufferHeight: 1440, quadrant: Int32(quadrant))
            for (u, v) in [(Float(0.5), Float(0.5)), (0.1, 0.2), (0.9, 0.35), (0.3, 0.95)] {
                let p = layout.picturePoint(fromBuffer: u, v)
                let back = layout.bufferPoint(fromVision: CGPoint(x: p.x / CGFloat(layout.width), y: 1 - p.y / CGFloat(layout.height)))
                XCTAssertEqual(back.u, u, accuracy: 1e-4)
                XCTAssertEqual(back.v, v, accuracy: 1e-4)
            }
            let k = layout.intrinsics(of: l2k_src_camera(fxn: 0.4, fyn: 0.6, cxn: 0.52, cyn: 0.47))
            let turned = quadrant & 1 == 1
            XCTAssertEqual(k.columns.0.x, 0.4 * 1920 * Float(layout.scale) * (turned ? 0.6 * 1440 / (0.4 * 1920) : 1), accuracy: 0.01)
            XCTAssertEqual(k.columns.1.y, 0.6 * 1440 * Float(layout.scale) * (turned ? 0.4 * 1920 / (0.6 * 1440) : 1), accuracy: 0.01)
            let centre = layout.picturePoint(fromBuffer: 0.52, 0.47)
            XCTAssertEqual(k.columns.2.x, Float(centre.x), accuracy: 0.01)
            XCTAssertEqual(k.columns.2.y, Float(centre.y), accuracy: 0.01)
        }
    }

    /// The drawn picture agrees with the layout: a mark in the camera buffer lands where `bufferPoint` says it came from,
    /// for every roll (this ties Core Image's turning of the buffer to the turning Vision's answers are mapped back with).
    func testVisionCanvasDrawsWhereTheLayoutSays() throws {
        let width = 400, height = 300
        var made: CVPixelBuffer?
        CVPixelBufferCreate(nil, width, height, kCVPixelFormatType_32BGRA, [kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary] as CFDictionary, &made)
        let buffer = try XCTUnwrap(made)
        let markX = 90, markY = 60, half = 6 // a white square away from every symmetry
        CVPixelBufferLockBaseAddress(buffer, [])
        let base = try XCTUnwrap(CVPixelBufferGetBaseAddress(buffer)).assumingMemoryBound(to: UInt8.self)
        let stride = CVPixelBufferGetBytesPerRow(buffer)
        for y in 0..<height {
            for x in 0..<width {
                let white = abs(x - markX) <= half && abs(y - markY) <= half
                for c in 0..<4 { base[y * stride + x * 4 + c] = c == 3 ? 255 : (white ? 255 : 0) }
            }
        }
        CVPixelBufferUnlockBaseAddress(buffer, [])
        let canvas = VisionCanvas()
        for quadrant in 0..<4 {
            let (picture, layout) = try XCTUnwrap(canvas.draw(buffer, quadrant: Int32(quadrant)))
            XCTAssertEqual(CVPixelBufferGetWidth(picture), layout.width)
            XCTAssertEqual(CVPixelBufferGetHeight(picture), layout.height)
            CVPixelBufferLockBaseAddress(picture, .readOnly)
            let pixels = try XCTUnwrap(CVPixelBufferGetBaseAddress(picture)).assumingMemoryBound(to: UInt8.self)
            let rowBytes = CVPixelBufferGetBytesPerRow(picture)
            var sumX = 0.0, sumY = 0.0, weight = 0.0
            for y in 0..<layout.height {
                for x in 0..<layout.width {
                    let value = Double(pixels[y * rowBytes + x * 4 + 1])
                    sumX += value * (Double(x) + 0.5)
                    sumY += value * (Double(y) + 0.5)
                    weight += value
                }
            }
            CVPixelBufferUnlockBaseAddress(picture, .readOnly)
            XCTAssertGreaterThan(weight, 0, "quadrant \(quadrant): the mark is not in the picture")
            let found = layout.bufferPoint(fromVision: CGPoint(x: sumX / weight / Double(layout.width), y: 1 - sumY / weight / Double(layout.height)))
            XCTAssertEqual(found.u, (Float(markX) + 0.5) / Float(width), accuracy: 0.005, "quadrant \(quadrant)")
            XCTAssertEqual(found.v, (Float(markY) + 0.5) / Float(height), accuracy: 0.005, "quadrant \(quadrant)")
        }
    }

    /// With a picture of a person (L2K_TEST_IMAGE): the camera detector survives the device being turned between
    /// portrait and landscape and a square camera, and Vision works with the focal length it is given.
    func testCameraDetectorOnAPictureThroughTurnsAndFormats() throws {
        guard let path = ProcessInfo.processInfo.environment["L2K_TEST_IMAGE"] else { throw XCTSkip("no L2K_TEST_IMAGE") }
        let image = try XCTUnwrap(CIImage(contentsOf: URL(fileURLWithPath: path)))
        let context = CIContext()
        // The picture as a camera of `width` × `height` would deliver it in a device rolled by `quadrant`.
        func cameraBuffer(_ width: Int, _ height: Int, quadrant: Int32) throws -> CVPixelBuffer {
            var made: CVPixelBuffer?
            CVPixelBufferCreate(nil, width, height, kCVPixelFormatType_32BGRA, [kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary] as CFDictionary, &made)
            let buffer = try XCTUnwrap(made)
            let turned = quadrant & 1 == 1
            let uw = CGFloat(turned ? height : width), uh = CGFloat(turned ? width : height)
            let scale = min(uw / image.extent.width, uh / image.extent.height)
            var upright = image.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
            upright = upright.transformed(by: CGAffineTransform(translationX: (uw - upright.extent.width) / 2 - upright.extent.minX,
                                                                y: (uh - upright.extent.height) / 2 - upright.extent.minY))
            upright = upright.composited(over: CIImage(color: .gray)).cropped(to: CGRect(x: 0, y: 0, width: uw, height: uh))
            let back: CGImagePropertyOrientation = [.up, .right, .down, .left][Int(quadrant & 3)] // undoes orientation(forQuadrant:)
            var sensor = upright.oriented(back)
            sensor = sensor.transformed(by: CGAffineTransform(translationX: -sensor.extent.minX, y: -sensor.extent.minY))
            context.render(sensor, to: buffer)
            return buffer
        }
        let detector = VisionHybridDetector()
        // (camera size, roll): the old 16:9 format, the 4:3 one, the square front camera; turned back and forth.
        let steps: [(Int, Int, Int32)] = [(1280, 720, 3), (1280, 720, 0), (1920, 1440, 3), (1920, 1440, 0), (1920, 1440, 1),
                                          (1280, 1280, 3), (1280, 1280, 0), (1920, 1440, 3)]
        var depths: [Float] = []
        for (width, height, quadrant) in steps {
            // a camera whose long side sees 100 degrees
            let focal = Float(max(width, height)) / 2 / tan(50 * .pi / 180)
            let camera = l2k_src_camera(fxn: focal / Float(width), fyn: focal / Float(height), cxn: 0.5, cyn: 0.5)
            let joints = detector.detect(in: try cameraBuffer(width, height, quadrant: quadrant), quadrant: quadrant, camera: camera)
            let root = joints.first { $0.source == Int(L2K_SRC_PELVIS) }
            print(String(format: "L2K-VISION camera %d×%d roll %d: %d joints, root %.2f m, focal given %.0f px, used %.0f px, error %@",
                         width, height, quadrant, joints.count, root?.depth ?? 0, detector.lastFocalGiven, detector.lastFocalUsed ?? 0, detector.lastError ?? "none"))
            XCTAssertNil(detector.lastError)
            XCTAssertFalse(joints.isEmpty, "\(width)×\(height) roll \(quadrant)")
            if let used = detector.lastFocalUsed { XCTAssertEqual(used, detector.lastFocalGiven, accuracy: detector.lastFocalGiven * 0.05) }
            if let root { depths.append(root.depth) }
        }
        print("L2K-VISION root depths: " + depths.map { String(format: "%.2f", $0) }.joined(separator: " "))
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
        let mixed = hybrid.detect(in: pixels, quadrant: 0, camera: l2k_src_camera(fxn: 0.6, fyn: 1.07, cxn: 0.5, cyn: 0.5))
        print("L2K-VISION hybrid joints: \(mixed.count), from the picture \(hybrid.lastSeen), from the model \(hybrid.lastModelOnly)")
        print("L2K-VISION hybrid " + mixed.map { String(format: "%d:(%.2f,%.2f,%.2fm,c%d)", $0.source, $0.u, $0.v, $0.depth, $0.confidence) }.joined(separator: " "))
    }

    /// With the colour stream switched off the sender does not offer it and never sends it, even to a receiver that asks.
    func testColourCanBeSwitchedOff() throws {
        XCTAssertEqual(L2KSession.capabilities(providesImages: true, colorEnabled: true), L2K.streamSkeleton | L2K.streamDepth | L2K.streamColor)
        XCTAssertEqual(L2KSession.capabilities(providesImages: true, colorEnabled: false), L2K.streamSkeleton | L2K.streamDepth)
        XCTAssertEqual(L2KSession.capabilities(providesImages: false, colorEnabled: true), L2K.streamSkeleton | L2K.streamDepth)

        let listener = try NWListener(using: .udp, on: .any)
        let queue = DispatchQueue(label: "test.receiver.nocolour")
        let gotDepth = expectation(description: "depth")
        gotDepth.assertForOverFulfill = false
        let lock = NSLock()
        var colourPackets = 0, offered: UInt32 = 0xFFFF_FFFF
        var seq: UInt32 = 0
        func receive(on connection: NWConnection) {
            connection.receiveMessage { data, _, _, error in
                if let data, !data.isEmpty {
                    data.withUnsafeBytes { raw in
                        let bytes = raw.bindMemory(to: UInt8.self).baseAddress!
                        var header = l2k_header()
                        guard l2k_decode_header(bytes, raw.count, &header) == 0 else { return }
                        switch Int32(header.type) {
                        case Int32(L2K_PKT_HELLO):
                            var hello = l2k_hello()
                            _ = l2k_decode_hello(bytes, raw.count, &hello)
                            lock.lock(); offered = hello.caps; lock.unlock()
                            var config = l2k_config()
                            config.streams = L2K.streamSkeleton | L2K.streamDepth | L2K.streamColor // asks for colour regardless
                            var packet = [UInt8](repeating: 0, count: 128)
                            seq += 1
                            let n = l2k_encode_config(&packet, 128, 77, seq, &config)
                            connection.send(content: Data(packet[0..<n]), completion: .idempotent)
                        case Int32(L2K_PKT_COLOR):
                            lock.lock(); colourPackets += 1; lock.unlock()
                        case Int32(L2K_PKT_DEPTH):
                            gotDepth.fulfill()
                        default: break
                        }
                    }
                }
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
        session.colorEnabled = false
        try session.start(SyntheticSource(players: 1))
        session.sender.connect(host: "127.0.0.1", port: port)
        wait(for: [gotDepth], timeout: 10)
        let settle = expectation(description: "one more second of streaming")
        DispatchQueue.global().asyncAfter(deadline: .now() + 1.2) { settle.fulfill() }
        wait(for: [settle], timeout: 5)
        session.sender.disconnect()
        session.stop()
        listener.cancel()
        lock.lock()
        defer { lock.unlock() }
        XCTAssertEqual(colourPackets, 0)
        XCTAssertEqual(offered, L2K.streamSkeleton | L2K.streamDepth)
    }

    /// Picture positions + bone lengths + "towards or away" give back the 3D body they came from.
    func testKinematicLiftRecoversTheBody() {
        let camera = l2k_src_camera(fxn: 0.38, fyn: 0.68, cxn: 0.5, cyn: 0.5) // an ultra-wide lens
        for t in [0.0, 0.4, 1.1, 1.7] {
            var truth = l2k_src_body()
            l2k_synth_body(t, 0, 1, &truth)
            let positions = withUnsafeBytes(of: truth.p) { Array($0.bindMemory(to: l2k_vec3.self)) }.map { SIMD3($0.x, $0.y, $0.z) }
            func image(_ p: SIMD3<Float>) -> (Float, Float) { (camera.cxn + camera.fxn * p.x / -p.z, camera.cyn - camera.fyn * p.y / -p.z) }
            var lengths: [Int: Float] = [:], away: [Int: Bool] = [:]
            for (joint, parent) in VisionKinematics.tree {
                lengths[joint] = simd_distance(positions[joint], positions[parent])
                away[joint] = VisionKinematics.isAway(parent: positions[parent], child: positions[joint])
            }
            // every joint arrives with the right picture position and a WRONG depth (the root excepted)
            let sources = [Int(L2K_SRC_PELVIS)] + VisionKinematics.tree.map(\.0)
            let joints = sources.map { source -> VisionJoint3D in
                let (u, v) = image(positions[source])
                return VisionJoint3D(source: source, u: u, v: v, depth: source == Int(L2K_SRC_PELVIS) ? -positions[source].z : 9)
            }
            let lifted = VisionKinematics.lift(joints, lengths: lengths, away: away, camera: camera)
            for joint in lifted {
                let p = VisionKinematics.point(u: joint.u, v: joint.v, depth: joint.depth, camera: camera)
                XCTAssertEqual(simd_distance(p, positions[joint.source]), 0, accuracy: 2e-3, "joint \(joint.source) at t=\(t)")
            }
            // a bone the picture says is longer than its length (the body is nearer than the lengths imply) does not blow up
            var short = lengths
            short[Int(L2K_SRC_WRIST_LEFT)] = 0.01
            let clipped = VisionKinematics.lift(joints, lengths: short, away: away, camera: camera)
            let wrist = clipped.first { $0.source == Int(L2K_SRC_WRIST_LEFT) }!
            XCTAssertTrue(wrist.depth.isFinite && wrist.depth > 0.5 && wrist.depth < 5)
        }
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

    /// The sender against `L2KReceiver` on localhost: the receiver's frames carry the sender's skeletons and depth image.
    func testReceiverGetsFramesFromSender() throws {
        let receiver = L2KReceiver()
        receiver.name = "unit game"
        receiver.streams = L2K.streamSkeleton | L2K.streamDepth | L2K.streamColor
        let listening = expectation(description: "listening")
        listening.assertForOverFulfill = false
        let receiving = expectation(description: "receiving")
        receiving.assertForOverFulfill = false
        let gotTracked = expectation(description: "tracked skeleton")
        gotTracked.assertForOverFulfill = false
        let gotDepth = expectation(description: "depth image")
        gotDepth.assertForOverFulfill = false
        let gotColor = expectation(description: "colour image")
        gotColor.assertForOverFulfill = false
        let lock = NSLock()
        var port: UInt16 = 0
        var lastStatus = L2KReceiver.Status()
        var numbers: [UInt32] = []
        receiver.onStatus = { status in
            lock.lock(); lastStatus = status; if status.port != 0 { port = status.port }; lock.unlock()
            if status.state == .listening { listening.fulfill() }
            if status.state == .receiving, status.senderName == "unit test" { receiving.fulfill() }
        }
        receiver.onFrame = { frame in
            lock.lock(); numbers.append(frame.skeleton.frame_number); lock.unlock()
            if frame.skeleton.trackedSkeletons.count == 2 { gotTracked.fulfill() }
            if let depth = frame.depth {
                XCTAssertEqual(depth.count, 320 * 240)
                if depth.filter({ l2k_depth_player($0) != 0 }).count > 1500 { gotDepth.fulfill() }
            }
            if let color = frame.color {
                XCTAssertEqual(color.extent.width, 640)
                XCTAssertEqual(color.extent.height, 480)
                gotColor.fulfill()
            }
        }
        receiver.start(port: 0, advertise: false)
        wait(for: [listening], timeout: 5)
        lock.lock(); let listenPort = port; lock.unlock()
        XCTAssertNotEqual(listenPort, 0)

        let session = L2KSession()
        session.deviceName = "unit test"
        let streaming = expectation(description: "sender streaming")
        streaming.assertForOverFulfill = false
        session.sender.onStatus = { status in
            if status.state == .streaming {
                XCTAssertEqual(status.receiverName, "unit game")
                streaming.fulfill()
            }
        }
        try session.start(SyntheticSource(players: 2))
        session.sender.connect(host: "127.0.0.1", port: listenPort)
        wait(for: [streaming, receiving, gotTracked, gotDepth, gotColor], timeout: 10)
        lock.lock()
        XCTAssertEqual(lastStatus.streams, L2K.streamSkeleton | L2K.streamDepth | L2K.streamColor)
        XCTAssertEqual(lastStatus.sourceKind, UInt8(L2K_SOURCE_SYNTHETIC))
        XCTAssertEqual(numbers, numbers.sorted(), "frames are delivered in order")
        XCTAssertEqual(Set(numbers).count, numbers.count, "no frame twice")
        lock.unlock()

        // the sender says BYE: the receiver hands out one frame with no bodies and goes back to listening
        let backToListening = expectation(description: "listening again")
        backToListening.assertForOverFulfill = false
        let emptied = expectation(description: "a frame without bodies")
        emptied.assertForOverFulfill = false
        receiver.onStatus = { if $0.state == .listening { backToListening.fulfill() } }
        receiver.onFrame = { frame in
            if frame.skeleton.slots.allSatisfy({ $0.state == 0 }), frame.depth?.allSatisfy({ $0 == 0 }) == true { emptied.fulfill() }
        }
        session.sender.disconnect()
        wait(for: [backToListening, emptied], timeout: 5)
        session.stop()
        receiver.stop()
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
