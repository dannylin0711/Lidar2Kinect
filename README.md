# Lidar2Kinect

Use an iPhone or iPad (camera + LiDAR) as the body tracker for two arcade dance games:

- **Dance Evolution Arcade**: the game thinks a Kinect v1 is attached. The device sends Kinect v1
  data: skeleton, depth + player index, colour.
- **DanceAround**: the game thinks its two RealSense cameras and VisionPose are attached. The device
  sends a 30-joint body at 60 frames a second. No RealSense, CUDA, TensorRT or VisionPose licence
  is needed, so it runs on any GPU.

The app streams over the local network (UDP 24810) to the Windows PC running the game with
spice2x. There a replacement DLL receives the stream: `Kinect10.dll` for Dance Evolution,
`visionposewrapper.dll` for DanceAround. The iPad port of Dance Evolution uses the same code
in-process.

## Download

[Releases](https://github.com/dannylin0711/Lidar2Kinect/releases); the latest is
[0.10.0](https://github.com/dannylin0711/Lidar2Kinect/releases/tag/v0.10.0).

| File | For | Where it goes |
|---|---|---|
| `Lidar2Kinect-<version>.ipa` | iPhone / iPad, iOS 17+ | unsigned: sideload it (Xcode, AltStore, Sideloadly, …) |
| `Kinect10.dll` (x86) | Dance Evolution Arcade | the game's `contents\` folder, next to `spice.exe` |
| `visionposewrapper.dll` (x64) | DanceAround | `game\dancearound_data\plugins\x86_64\`; rename the original to `visionposewrapper.dll.orig` first |

Set up:

1. Allow UDP 24810 inbound through the Windows firewall for the game's executable
   (`windows/vp4u/README.md` "Firewall"). Without it the phone seems to send nothing.
2. On the iPhone: Settings → Privacy & Security → **Local Network** → Lidar2Kinect on.
3. Start the game, then the app, and pick the PC.

Where to stand:
- **Dance Evolution:** 1.8–3.2 m from the phone (`docs/01-dea-kinect-surface.md` §7).
- **DanceAround:** about 2.3 m, with the phone about 1 m up (`windows/vp4u/README.md` "Where to put
  the phone").

## What is here

| Folder | What | State |
|---|---|---|
| `shared/` | Portable C library: the wire protocol and the Kinect translation. The contract between both sides. | working, tested |
| `ios/` | `L2KKit` (Swift) and the universal iOS / iPadOS app, `ios/Lidar2Kinect.xcodeproj`, app 0.10.0 | runs on iPhone and iPad (LiDAR, ultra-wide and front-camera sources); used to play Dance Evolution. Its 60 frames a second and the 30-joint body have not yet been tried on a device. |
| `windows/` | Dance Evolution: `Kinect10.dll` and a spice2x hook (x86) | in the game with the test sender; see `windows/README.md`, `windows/STATUS.md` |
| `windows/vp4u/` | DanceAround: `visionposewrapper.dll` (x64), the vp4u API 210 fed by the same stream | in the game with the test sender (player calibration, songs played to the end); not yet with the app; see `windows/vp4u/README.md` |
| `docs/` | Overview, what the games need, protocol, conventions, status | |

Start with `docs/00-overview.md`. The latest state is in `docs/04-status.md`.

## Build and test

Shared library and tools (macOS / Linux):

```sh
make -C shared test          # tests against the golden packets
make -C shared               # also builds build/shared/l2k_dump and l2k_fake_sender
```

Swift package:

```sh
swift test
```

App:

```sh
cd ios && xcodegen generate   # only after editing project.yml
open Lidar2Kinect.xcodeproj   # device builds need ios/Signing.xcconfig (see Signing.xcconfig.example)
```

Windows (Visual Studio 2022): Dance Evolution, 32-bit:

```
cmake -S windows -B build/windows -A Win32
cmake --build build/windows --config Release
ctest --test-dir build/windows -C Release --output-on-failure
```

DanceAround, 64-bit:

```
cmake -S windows/vp4u -B build/vp4u -A x64
cmake --build build/vp4u --config Release
ctest --test-dir build/vp4u -C Release --output-on-failure
```

Try it without a camera or a PC, with a receiver and a sender on one machine:

```sh
build/shared/l2k_dump &
build/shared/l2k_fake_sender -n 2 -j shared/tests/vectors/testcard_640x480.jpg 127.0.0.1
```

The app in the simulator does the same with the launch arguments
`-l2kSource synthetic -l2kHost 127.0.0.1 -l2kAutoConnect 1`. `l2k_fake_sender` also feeds either
game in place of the phone.

## Not included

No game files, no Konami code, no Microsoft SDK files. The game dumps and the reverse-engineering
projects live outside this repository.
