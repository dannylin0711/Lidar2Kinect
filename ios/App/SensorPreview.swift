// What the virtual Kinect sees: the colour or depth image it produces, with the Kinect skeleton drawn on top.
import L2KKit
import L2KShared
import SwiftUI

struct SensorPreview: View {
    let image: CGImage?
    let skeleton: l2k_skeleton_frame?

    private static let slotColors: [Color] = [.green, .cyan, .yellow, .pink, .orange, .mint]

    var body: some View {
        Canvas { context, size in
            let frame = CGRect(origin: .zero, size: size)
            context.fill(Path(frame), with: .color(.black))
            if let image {
                context.draw(Image(decorative: image, scale: 1), in: frame)
            }
            guard let skeleton else { return }
            for (slot, body) in skeleton.trackedSkeletons {
                let color = Self.slotColors[slot % Self.slotColors.count]
                let positions = body.jointPositions
                let states = body.jointStates
                // skeleton space -> depth image pixel -> view point (the same projection the receiver side uses)
                let points: [CGPoint?] = positions.map { p in
                    var u: Float = 0, v: Float = 0
                    guard l2k_skeleton_to_depth_pixel(p, 320, 240, &u, &v) == 1 else { return nil }
                    return CGPoint(x: CGFloat(u) / 320 * size.width, y: CGFloat(v) / 240 * size.height)
                }
                for (a, b) in KinectJoint.bones {
                    guard let pa = points[a.rawValue], let pb = points[b.rawValue] else { continue }
                    var path = Path()
                    path.move(to: pa)
                    path.addLine(to: pb)
                    context.stroke(path, with: .color(color), lineWidth: 3)
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
