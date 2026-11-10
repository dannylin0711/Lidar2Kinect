// What the virtual Kinect sees: the colour or depth image it produces, with the Kinect skeleton drawn on top.
import L2KKit
import L2KShared
import SwiftUI

struct SensorPreview: View {
    let image: CGImage?
    let skeleton: l2k_skeleton_frame?
    /// The picture shows this many times the Kinect's field of view; the Kinect's own image is the framed middle.
    var zoom: CGFloat = 1

    private static let slotColors: [Color] = [.green, .cyan, .yellow, .pink, .purple, .mint]

    /// A small view from above in the top-right corner: what the game is told about where the player is and how far
    /// each limb reaches towards the sensor - the part a front view cannot show. The sensor is at the bottom, the
    /// frame is the game's play area (1.8-3.2 m away, 1.25 m either side).
    private func drawTopView(_ skeleton: l2k_skeleton_frame, in context: inout GraphicsContext, size: CGSize) {
        let box = CGRect(x: size.width * 0.70, y: size.height * 0.03, width: size.width * 0.27, height: size.width * 0.27 * 4.0 / 3.6)
        context.fill(Path(roundedRect: box, cornerRadius: 6), with: .color(.black.opacity(0.55)))
        // metres -> points: x from -1.8 to +1.8 (mirrored like the picture), distance 0 (bottom) to 4 m (top)
        func point(_ p: l2k_vec3) -> CGPoint {
            CGPoint(x: box.midX + CGFloat(p.x) / 3.6 * box.width, y: box.maxY - CGFloat(p.z) / 4.0 * box.height)
        }
        let area = CGRect(origin: point(l2k_vec3(x: -1.25, y: 0, z: 3.2)),
                          size: CGSize(width: 2.5 / 3.6 * box.width, height: 1.4 / 4.0 * box.height))
        context.stroke(Path(area), with: .color(.white.opacity(0.5)), lineWidth: 1)
        context.fill(Path(ellipseIn: CGRect(x: box.midX - 3, y: box.maxY - 6, width: 6, height: 6)), with: .color(.white))
        for (slot, body) in skeleton.trackedSkeletons {
            let color = Self.slotColors[slot % Self.slotColors.count]
            for (a, b) in KinectJoint.bones {
                var path = Path()
                path.move(to: point(body.position(of: a)))
                path.addLine(to: point(body.position(of: b)))
                let side = Self.side(of: b)
                context.stroke(path, with: .color(side < 0 ? .orange : side > 0 ? .blue : color), lineWidth: 1.5)
            }
        }
    }

    /// -1 = a joint of the player's left side, +1 = right side, 0 = the middle.
    private static func side(of joint: KinectJoint) -> Int {
        switch joint {
        case .shoulderLeft, .elbowLeft, .wristLeft, .handLeft, .hipLeft, .kneeLeft, .ankleLeft, .footLeft: return -1
        case .shoulderRight, .elbowRight, .wristRight, .handRight, .hipRight, .kneeRight, .ankleRight, .footRight: return 1
        default: return 0
        }
    }

    var body: some View {
        Canvas { context, size in
            let frame = CGRect(origin: .zero, size: size)
            context.fill(Path(frame), with: .color(.black))
            if let image {
                context.draw(Image(decorative: image, scale: 1), in: frame)
            }
            if zoom > 1 {
                // what the Kinect's own image covers
                let inner = CGRect(x: size.width * (0.5 - 0.5 / zoom), y: size.height * (0.5 - 0.5 / zoom),
                                   width: size.width / zoom, height: size.height / zoom)
                context.stroke(Path(inner), with: .color(.white.opacity(0.6)), style: StrokeStyle(lineWidth: 1, dash: [6, 4]))
            }
            guard let skeleton else { return }
            drawTopView(skeleton, in: &context, size: size)
            for (slot, body) in skeleton.trackedSkeletons {
                let color = Self.slotColors[slot % Self.slotColors.count]
                let positions = body.jointPositions
                let states = body.jointStates
                // skeleton space -> depth image pixel -> view point (the same projection the receiver side uses)
                let points: [CGPoint?] = positions.map { p in
                    var u: Float = 0, v: Float = 0
                    guard l2k_skeleton_to_depth_pixel(p, 320, 240, &u, &v) == 1 else { return nil }
                    // a zoomed-out picture shrinks everything towards the centre
                    return CGPoint(x: (0.5 + (CGFloat(u) / 320 - 0.5) / zoom) * size.width,
                                   y: (0.5 + (CGFloat(v) / 240 - 0.5) / zoom) * size.height)
                }
                for (a, b) in KinectJoint.bones {
                    guard let pa = points[a.rawValue], let pb = points[b.rawValue] else { continue }
                    var path = Path()
                    path.move(to: pa)
                    path.addLine(to: pb)
                    // The player's own left side is orange, right side blue, the middle the slot's colour:
                    // raise your left hand and the orange arm must go up.
                    let side = Self.side(of: b)
                    context.stroke(path, with: .color(side < 0 ? .orange : side > 0 ? .blue : color), lineWidth: 3)
                }
                for (index, point) in points.enumerated() {
                    guard let point else { continue }
                    // tracked joints are solid, inferred ones hollow
                    let dot = Path(ellipseIn: CGRect(x: point.x - 4, y: point.y - 4, width: 8, height: 8))
                    if states[index] == UInt8(L2K_JOINT_TRACKED) {
                        context.fill(dot, with: .color(.white))
                    } else {
                        context.stroke(dot, with: .color(.white), lineWidth: 1.5)
                    }
                }
            }
        }
        .aspectRatio(4.0 / 3.0, contentMode: .fit)
        .clipShape(RoundedRectangle(cornerRadius: 12))
    }
}
