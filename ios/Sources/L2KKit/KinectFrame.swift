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
    /// 30-joint bodies (Kinect v2's joints and five face points; DanceAround's receiver asks for them).
    public static let streamBody = UInt32(L2K_STREAM_BODY)
    /// Skeleton frames per second when the receiver asks for no rate (a Kinect's rate).
    public static let defaultRate = Double(L2K_DEFAULT_RATE)

    /// Microseconds on the monotonic clock the frame timestamps use (the system uptime clock).
    public static func nowMicroseconds() -> UInt64 {
        UInt64(ProcessInfo.processInfo.systemUptime * 1_000_000)
    }
}

/// The game a receiver feeds, from its CONFIG consumer kind (`l2k_game_of_consumer`). Each has its own play area and
/// standing point (docs/03 "Play area").
public enum L2KGame: Int32, CaseIterable {
    case danceEvolution = 0 // L2K_GAME_DANCE_EVOLUTION
    case danceAround = 1    // L2K_GAME_DANCEAROUND

    public init(consumerKind: UInt8) {
        self = L2KGame(rawValue: l2k_game_of_consumer(consumerKind)) ?? .danceEvolution
    }

    public var name: String { self == .danceAround ? "DanceAround" : "Dance Evolution" }

    /// The mapper's play area and standing point for this game (the other fields are left alone).
    func apply(to params: inout l2k_map_params) { l2k_map_params_set_game(&params, rawValue) }

    private var params: l2k_map_params {
        var p = l2k_map_params()
        l2k_map_params_default(&p)
        apply(to: &p)
        return p
    }

    /// Where the game takes a player (metres from the sensor, level; sideways either side of its axis): the area the
    /// game reads, not the 10 cm smaller one the fit keeps to.
    public var playAreaNear: Float { params.area_near - 0.10 }
    public var playAreaFar: Float { params.area_far + 0.10 }
    public var playAreaHalfWidth: Float { params.area_half_width + 0.10 }
    /// Where "Best position" puts a single player (metres from the sensor).
    public var standingDistance: Float { params.norm_distance }
    /// The game can take two players at once (DanceAround takes the nearest body only).
    public var takesTwoPlayers: Bool { self == .danceEvolution }
}

public extension l2k_body_frame {
    /// The six body slots.
    var slots: [l2k_body] {
        withUnsafeBytes(of: bodies) { Array($0.bindMemory(to: l2k_body.self)) }
    }

    /// Slots that hold a fully tracked body, with their slot index.
    var trackedBodies: [(slot: Int, body: l2k_body)] {
        slots.enumerated().compactMap { $0.element.state == UInt8(L2K_SKELETON_TRACKED) ? (slot: $0.offset, body: $0.element) : nil }
    }
}

public extension l2k_body {
    var jointPositions: [l2k_vec3] {
        withUnsafeBytes(of: joints) { Array($0.bindMemory(to: l2k_vec3.self)) }
    }

    var jointStates: [UInt8] {
        withUnsafeBytes(of: joint_state) { Array($0) }
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
