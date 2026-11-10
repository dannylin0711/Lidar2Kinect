// swift-tools-version: 5.9
// Lidar2Kinect - Swift package for the Apple side.
//   L2KShared : the portable C library in shared/ (protocol, Kinect types, mapping). The Windows side builds the same
//               sources with shared/CMakeLists.txt.
//   L2KKit    : capture (ARKit / synthetic), the frame pipeline and the network sender, in Swift.
// The Lidar2Kinect app (ios/) and the Dance Evolution iPad port both depend on this package.
import PackageDescription

let package = Package(
    name: "Lidar2Kinect",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "L2KShared", targets: ["L2KShared"]),
        .library(name: "L2KKit", targets: ["L2KKit"]),
    ],
    targets: [
        .target(
            name: "L2KShared",
            path: "shared",
            exclude: ["tests", "tools", "Makefile", "CMakeLists.txt"],
            sources: ["src"],
            publicHeadersPath: "include"
        ),
        .target(
            name: "L2KKit",
            dependencies: ["L2KShared"],
            path: "ios/Sources/L2KKit"
        ),
        .testTarget(
            name: "L2KKitTests",
            dependencies: ["L2KKit", "L2KShared"],
            path: "ios/Tests/L2KKitTests"
        ),
    ]
)
