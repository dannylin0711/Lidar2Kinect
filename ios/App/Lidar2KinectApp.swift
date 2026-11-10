import SwiftUI

@main
struct Lidar2KinectApp: App {
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
