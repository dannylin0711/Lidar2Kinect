# Lidar2Kinect

Use an iPhone or iPad (camera + LiDAR) as the Kinect for **Dance Evolution Arcade**.

The device tracks the player and outputs Kinect v1 data — skeleton, depth + player index, colour.
A Windows PC running the game with spice2x receives it over the local network through a
replacement `Kinect10.dll` or a spice2x hook; the iPad port of the game uses the same code
in-process.

| Folder | What | State |
|---|---|---|
| `shared/` | Portable C library: the wire protocol and the Kinect translation. The contract between both sides. | working, tested |
| `ios/` | `L2KKit` (Swift) and the universal iOS / iPadOS app, `ios/Lidar2Kinect.xcodeproj` | builds, streams in the simulator; not yet run on a device |
| `windows/` | Fake `Kinect10.dll` and spice2x hook | working with the fake sender, in the game — see `windows/README.md`, `windows/STATUS.md` |
| `windows/vp4u/` | **DanceAround**: replacement `visionposewrapper.dll` (x64) fed by the same stream | builds, tests pass with the fake sender; not yet run in the game — `windows/vp4u/README.md` |
| `docs/` | Overview, what the game needs, protocol, conventions, status | |

Start with `docs/00-overview.md`.

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

Try it without a camera or a PC — a receiver and a sender on one machine:

```sh
build/shared/l2k_dump &
build/shared/l2k_fake_sender -n 2 -j shared/tests/vectors/testcard_640x480.jpg 127.0.0.1
```

The app in the simulator does the same with the launch arguments
`-l2kSource synthetic -l2kHost 127.0.0.1 -l2kAutoConnect 1`.

## Not included

No game files, no Konami code, no Microsoft SDK files. The game dump and the reverse-engineering
project live outside this repository.
