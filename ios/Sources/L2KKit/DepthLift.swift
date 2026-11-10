// Lifting 2D body joints into 3D with a depth map. Pure buffer maths (no ARKit), so it is tested on the Mac.
import Foundation
import L2KShared

/// A depth map (metres along the optical axis, Float32 rows) and an optional person mask (UInt8 rows, >= 128 = person),
/// both registered to the camera image.
struct DepthLiftBuffers {
    var depth: UnsafeRawPointer
    var depthWidth: Int
    var depthHeight: Int
    var depthStride: Int
    var mask: UnsafeRawPointer?
    var maskWidth = 0
    var maskHeight = 0
    var maskStride = 0
}

/// One detected 2D joint: its source joint index (L2K_SRC_*), its normalised image position (origin top-left) and the
/// distance from the body surface the depth sensor sees to the joint inside the body (metres).
struct DepthLiftJoint {
    var source: Int
    var u: Float
    var v: Float
    var radius: Float
}

enum DepthLift {
    /// Joints whose depth defines "where the body is".
    static let coreJoints: Set<Int> = [Int(L2K_SRC_PELVIS), Int(L2K_SRC_NECK), Int(L2K_SRC_SHOULDER_LEFT),
                                       Int(L2K_SRC_SHOULDER_RIGHT), Int(L2K_SRC_HIP_LEFT), Int(L2K_SRC_HIP_RIGHT)]
    /// A limb joint further than this from the body's depth took a wrong depth (metres).
    static let reach: Float = 1.0

    /// Depth of the body surface at a normalised image point: the median over the person pixels of a 5x5 window,
    /// or - when the mask has none there - the nearer quarter of the window (the body is in front of the room).
    static func surfaceDepth(_ b: DepthLiftBuffers, u: Float, v: Float) -> (depth: Float, onPerson: Bool)? {
        let cx = Int(u * Float(b.depthWidth)), cy = Int(v * Float(b.depthHeight))
        var person: [Float] = [], all: [Float] = []
        for y in (cy - 2)...(cy + 2) where y >= 0 && y < b.depthHeight {
            let row = b.depth.advanced(by: y * b.depthStride).assumingMemoryBound(to: Float.self)
            for x in (cx - 2)...(cx + 2) where x >= 0 && x < b.depthWidth {
                let d = row[x]
                guard d.isFinite, d > 0.2, d < 8 else { continue }
                all.append(d)
                if let mask = b.mask {
                    let mx = min(b.maskWidth - 1, x * b.maskWidth / b.depthWidth)
                    let my = min(b.maskHeight - 1, y * b.maskHeight / b.depthHeight)
                    if mask.advanced(by: my * b.maskStride + mx).assumingMemoryBound(to: UInt8.self).pointee >= 128 { person.append(d) }
                }
            }
        }
        if !person.isEmpty {
            person.sort()
            return (person[person.count / 2], true)
        }
        guard !all.isEmpty else { return nil }
        all.sort()
        return (all[all.count / 4], false)
    }

    /// Place the joints in source camera space (+x image right, +y image up, the camera looks along -z).
    /// Returns the number of joints placed.
    static func lift(_ joints: [DepthLiftJoint], buffers: DepthLiftBuffers, camera: l2k_src_camera, into body: inout l2k_src_body) -> Int {
        var found: [(joint: DepthLiftJoint, depth: Float, confidence: UInt8)] = []
        for joint in joints {
            guard joint.u.isFinite, joint.v.isFinite, joint.u >= 0, joint.u < 1, joint.v >= 0, joint.v < 1 else { continue }
            guard let surface = surfaceDepth(buffers, u: joint.u, v: joint.v) else { continue }
            let sure = surface.onPerson || buffers.mask == nil
            found.append((joint, surface.depth + joint.radius, sure ? 2 : 1))
        }
        // A limb joint whose depth is far from the torso's took the background (or something in front): hold it at
        // the torso's depth and report it as estimated.
        let core = found.filter { coreJoints.contains($0.joint.source) }.map(\.depth).sorted()
        let reference = core.isEmpty ? nil : core[core.count / 2]
        for item in found {
            var d = item.depth
            var confidence = item.confidence
            if let reference, abs(d - reference) > reach {
                d = reference
                confidence = 1
            }
            // pinhole: image x right, y down
            let x = (item.joint.u - camera.cxn) / camera.fxn * d
            let y = -(item.joint.v - camera.cyn) / camera.fyn * d
            body.set(item.joint.source, SIMD3(x, y, -d), confidence: confidence)
        }
        return found.count
    }
}
