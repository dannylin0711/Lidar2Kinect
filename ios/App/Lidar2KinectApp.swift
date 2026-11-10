import SwiftUI

import ARKit

@main
struct Lidar2KinectApp: App {
    init() {
        // -l2kDumpJoints 1: print ARKit's skeleton joint names (works in the simulator too), then carry on.
        if UserDefaults.standard.bool(forKey: "l2kDumpJoints") {
            for (label, definition) in [("2D", ARSkeletonDefinition.defaultBody2D), ("3D", ARSkeletonDefinition.defaultBody3D)] {
                print("L2K-JOINTS \(label) count=\(definition.jointCount): \(definition.jointNames.joined(separator: ","))")
            }
        }
    }

    // The model is built inside the StateObject autoclosure, never in a view's init.
    @StateObject private var model = SensorModel()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(model)
                .preferredColorScheme(.dark)
                .onAppear {
                    UIApplication.shared.isIdleTimerDisabled = true // the sensor must not go to sleep mid-song
                    model.startIfNeeded()
                }
        }
    }
}
