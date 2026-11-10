// Swift conveniences over the C types of the shared library (fixed C arrays import as tuples).
import Foundation
import L2KShared

/// The 20 Kinect v1 joints, in NUI order. LEFT / RIGHT are the player's own left / right.
public enum KinectJoint: Int, CaseIterable {
    case hipCenter = 0, spine, shoulderCenter, head
    case shoulderLeft, elbowLeft, wristLeft, handLeft
    case shoulderRight, elbowRight, wristRight, handRight
    case hipLeft, kneeLeft, ankleLeft, footLeft
    case hipRight, kneeRight, ankleRight, footRight

    /// The bones of the stick figure, for drawing.
    public static let bones: [(KinectJoint, KinectJoint)] = [
        (.hipCenter, .spine), (.spine, .shoulderCenter), (.shoulderCenter, .head),
        (.shoulderCenter, .shoulderLeft), (.shoulderLeft, .elbowLeft), (.elbowLeft, .wristLeft), (.wristLeft, .handLeft),
        (.shoulderCenter, .shoulderRight), (.shoulderRight, .elbowRight), (.elbowRight, .wristRight), (.wristRight, .handRight),
        (.hipCenter, .hipLeft), (.hipLeft, .kneeLeft), (.kneeLeft, .ankleLeft), (.ankleLeft, .footLeft),
        (.hipCenter, .hipRight), (.hipRight, .kneeRight), (.kneeRight, .ankleRight), (.ankleRight, .footRight),
    ]
}

public enum L2K {
    public static let depthWidth = Int(L2K_DEPTH_WIDTH)
    public static let depthHeight = Int(L2K_DEPTH_HEIGHT)
    public static let colorWidth = Int(L2K_COLOR_WIDTH)
    public static let colorHeight = Int(L2K_COLOR_HEIGHT)
    public static let defaultPort = UInt16(L2K_DEFAULT_PORT)
    public static let bonjourType = "_l2k._udp"
    public static let maxDatagram = Int(L2K_HEADER_SIZE) + Int(L2K_CHUNK_HEADER_SIZE) + Int(L2K_CHUNK_PAYLOAD)

    public static let streamSkeleton = UInt32(L2K_STREAM_SKELETON)
    public static let streamDepth = UInt32(L2K_STREAM_DEPTH)
    public static let streamColor = UInt32(L2K_STREAM_COLOR)

    /// Microseconds on the monotonic clock the frame timestamps use (the system uptime clock).
    public static func nowMicroseconds() -> UInt64 {
        UInt64(ProcessInfo.processInfo.systemUptime * 1_000_000)
    }
}

public extension l2k_skeleton_frame {
    /// The six skeleton slots.
    var slots: [l2k_skeleton] {
        withUnsafeBytes(of: skeletons) { Array($0.bindMemory(to: l2k_skeleton.self)) }
    }

    /// Slots that hold a fully tracked skeleton, with their slot index.
    var trackedSkeletons: [(slot: Int, skeleton: l2k_skeleton)] {
        slots.enumerated().compactMap { $0.element.isTracked ? (slot: $0.offset, skeleton: $0.element) : nil }
    }

    /// Sensor tilt in degrees, positive = looking upwards.
    var elevationDegrees: Float { l2k_elevation_degrees(up) }
}

public extension l2k_skeleton {
    var isTracked: Bool { state == UInt8(L2K_SKELETON_TRACKED) }

    var jointPositions: [l2k_vec3] {
        withUnsafeBytes(of: joints) { Array($0.bindMemory(to: l2k_vec3.self)) }
    }

    var jointStates: [UInt8] {
        withUnsafeBytes(of: joint_state) { Array($0) }
    }

    func position(of joint: KinectJoint) -> l2k_vec3 {
        withUnsafeBytes(of: joints) { $0.bindMemory(to: l2k_vec3.self)[joint.rawValue] }
    }
}

public extension l2k_src_body {
    /// Set one source joint (index = L2K_SRC_*), position in source camera space.
    mutating func set(_ joint: Int, _ position: SIMD3<Float>, confidence: UInt8) {
        withUnsafeMutableBytes(of: &p) {
            $0.bindMemory(to: l2k_vec3.self)[joint] = l2k_vec3(x: position.x, y: position.y, z: position.z)
        }
        withUnsafeMutableBytes(of: &conf) { $0[joint] = confidence }
    }
}

/// Copy a Swift string into a fixed `char[]` field (NUL padded, always terminated).
func l2kSetName<T>(_ field: inout T, _ string: String) {
    withUnsafeMutableBytes(of: &field) { buffer in
        for i in 0..<buffer.count { buffer[i] = 0 }
        for (i, byte) in string.utf8.prefix(buffer.count - 1).enumerated() { buffer[i] = byte }
    }
}

/// Read a fixed `char[]` field as a string.
func l2kGetName<T>(_ field: T) -> String {
    withUnsafeBytes(of: field) { buffer in
        let bytes = buffer.prefix { $0 != 0 }
        return String(decoding: bytes, as: UTF8.self)
    }
}
