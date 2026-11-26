# Working in ios/ (the Apple side)

The root `CLAUDE.md` rules apply. This file says what the app's stream now feeds on Windows and
what the Windows side asks of the app next. Written from the Windows side at Danny's request
(2026-11-26); keep it current from here on.

## Two games now read the stream

| Game | Windows receiver | Streams it asks for | State |
|---|---|---|---|
| Dance Evolution Arcade (x86) | `windows/kinect10`, `windows/spicehook` | skeleton, depth, colour | working in the game with `l2k_fake_sender` |
| DanceAround (x64) | `windows/vp4u` — replaces the game's `VisionPoseWrapper.dll` | skeleton only | the game boots to its attract demo with it; not yet fed a skeleton in the game |

Both take today's L2K1 stream unchanged: nothing in `shared/` or `ios/` changed for DanceAround.
Its CONFIG carries `consumer_kind` 0 (unknown) and the PC's name. Details:
`windows/vp4u/README.md`, `windows/docs/vp4u-interface.md`, `docs/04-status.md`.

## What DanceAround relies on — do not break

`windows/vp4u` turns each SKELETON frame into the game's 30-joint bodies (`vp4u_map.cpp`):

- **`up`** must be true gravity-up in skeleton space: the receiver levels the skeleton with it
  (the phone's pitch and roll are removed there, not on the phone).
- **Floor plane `D`** (sensor height, with `L2K_FRAME_FLOOR_VALID`) places the skeleton
  vertically: the game's origin is the cabinet camera, 1.16 m above the floor.
- **Axes and units of `docs/03`** exactly: +x the player's right when facing the phone, +y up,
  +z away from the phone towards the player, metres. Depth matters most: the game only sees a
  player whose hips are **2.3 ± 0.5 m** from the phone (play area of the game's config).
- Stable `tracking_id`; joint states (the game treats inferred as valid).
- Only fully tracked bodies are used; 30 fps works (frames are passed on as they arrive).

## Next, in order

1. **Run on a real LiDAR device** against a Windows receiver — still the first real-device run for
   either game. Report: missing ARKit joint names (`ARKitBodySource.statusText`), frame rate,
   latency, whether `up` and the floor height are right (a level phone 1 m up should report
   ~1.0 m).
2. **60 fps for DanceAround.** Its cameras ran at 60; ARKit gives 60; `L2KSession.outputRate`
   is 30 because a Kinect is. Dance Evolution must stay at 30. So the rate has to be per
   receiver — an app setting is enough for a first try; asking for it in CONFIG is a protocol
   decision (see below).
3. **A 30-joint body stream.** DanceAround's skeleton is Kinect v2's 25 joints + nose, eyes, ears
   (order in `windows/vp4u/vp4u_abi.h`). The Windows side now derives 10 of them from the 20
   Kinect v1 joints (`windows/docs/vp4u-interface.md` §6). ARKit has most of them itself:
   thumb and finger-tip joints (`*_handThumb*`, `*_handMid*End`), `nose_joint`, eye joints, more
   neck/spine joints — check the names on a device; there are no ear joints in the 3D skeleton.
   Proposal: a new stream bit and packet type with that joint set, same space and conventions
   as SKELETON. Old receivers never ask for it.
4. A `consumer_kind` value for the vp4u receiver, so the app can show which game it feeds.

2–4 change `shared/` (a stream bit, a packet type, a CONFIG field or value): additions only, so
old peers are unaffected, but follow the root `CLAUDE.md` (tests, golden packets, `docs/02`, a
`docs/04-status.md` entry) and settle the layout with the Windows side first — `windows/vp4u`
is its consumer.

## Placement text for the app (DanceAround)

Phone centred in front of the play area, rear camera towards the player, landscape, about 1 m
up; the player stands about 2.3 m away.
