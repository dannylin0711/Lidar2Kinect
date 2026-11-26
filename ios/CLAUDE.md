# Working in ios/ (the Apple side)

The root `CLAUDE.md` rules apply. This file says what the app's stream now feeds on Windows and
what the Windows side asks of the app next. Written from the Windows side at Danny's request
(2026-11-26, after rebasing onto app 0.9.4); keep it current from here on.

## Two games now read the stream

| Game | Windows receiver | Streams it asks for | State |
|---|---|---|---|
| Dance Evolution Arcade (x86) | `windows/kinect10`, `windows/spicehook` | skeleton, depth, colour | working in the game (fake sender, and Danny's own play with the app) |
| DanceAround (x64) | `windows/vp4u` — replaces the game's `VisionPoseWrapper.dll` | skeleton only | **playable with `l2k_fake_sender`**: the game's player calibration measures the figure mid play area, two songs played to the result screen at 30 pose frames/s; **not yet run with the app** |

DanceAround takes today's L2K1 stream unchanged: nothing in `shared/` or `ios/` changed for it.
Its CONFIG carries `consumer_kind` 0 (unknown) and the PC's name; it advertises `_l2k._udp`.
Details: `windows/vp4u/README.md`, `windows/docs/vp4u-interface.md`, `docs/04-status.md`.

## What DanceAround relies on — do not break

`windows/vp4u` turns each SKELETON frame into the game's 30-joint bodies (`vp4u_map.cpp`):

- **`up`** must be true gravity-up in skeleton space: the receiver levels the skeleton with it
  (the phone's pitch and roll are removed there). With "Best position" on, the frame is already
  level, which is fine.
- **Floor plane `D`** (sensor height, with `L2K_FRAME_FLOOR_VALID`) places the skeleton
  vertically: the game's origin is the cabinet camera, 1.16 m above the floor.
- **Axes and units of `docs/03`** exactly: +x the player's right when facing the phone, +y up,
  +z away from the phone towards the player, metres.
- **Real positions.** The game only sees a player whose hips (SpineBase) are inside its play area:
  **1.80–2.81 m from the phone, ±0.9 m sideways** of the phone's axis (the receiver puts the
  phone at the play area's centre line; `phone_x` / `phone_z` in `l2k.ini` move it).
- Stable `tracking_id`; joint states (the game treats inferred as valid). Only fully tracked bodies
  are used; the game takes the nearest one.

### The app's Dance Evolution options and DanceAround

Both are tuned for Dance Evolution's play area, and the app cannot yet tell which game it feeds:

- **"Fit me into the game's play area"** (on by default): slides a body outside 1.80–3.20 m /
  ±1.25 m to just inside. For DanceAround it helps a player who stands too close, but someone at
  2.9–3.2 m or more than 0.9 m to the side stays outside DanceAround's area and is not seen.
- **"Best position for Dance Evolution"** (off by default): level frame, player at 2.5 m, centred —
  inside DanceAround's area, so usable as is. With 2 players, DanceAround takes only the nearer.

## Next, in order

1. **Run the app against DanceAround.** The Windows side is installed on Danny's PC (spice2x,
   `F:\BEMANI\UDN-001-2022062801`); the app finds it by Bonjour or the PC's address, UDP 24810.
   Stand about 2.3 m from the phone. Watch: the game's log `l2k: sender "…" connected`, its
   player calibration (`GamePlayerCalib:result pos (x, y, z)` should be near (0.55, −0.2, 2.3)),
   and **left/right**: raise the right arm, the avatar should mirror it like a mirror would (if
   not, `[vp4u] mirror_x=1` on Windows — tell the Windows side, it is open point 1 there).
2. **A `consumer_kind` for the vp4u receiver** (e.g. `L2K_CONSUMER_VP4U = 5`), so the app can apply
   the right play area — DanceAround's box above, or none — instead of Dance Evolution's.
3. **60 fps for DanceAround.** Its cameras ran at 60; the app now captures at 60 on the ultra-wide
   lens but `L2KSession.outputRate` is 30, which Dance Evolution must keep. Per receiver: an app
   setting is enough to start; asking for it in CONFIG is a protocol decision.
4. **A 30-joint body stream.** DanceAround's skeleton is Kinect v2's 25 joints + nose, eyes, ears
   (order in `windows/vp4u/vp4u_abi.h`). The Windows side derives 10 of them from the 20 Kinect v1
   joints (`windows/docs/vp4u-interface.md` §6). ARKit and Vision have most of them (thumbs, finger
   tips, nose, eyes; Vision 2D also ears). Proposal: a new stream bit and packet type with that
   joint set, same space and conventions as SKELETON; old receivers never ask for it.

2–4 change `shared/` (a stream bit, a packet type, a CONFIG field or value): additions only, so
old peers are unaffected, but follow the root `CLAUDE.md` (tests, golden packets, `docs/02`, a
`docs/04-status.md` entry) and settle the layout with the Windows side first — `windows/vp4u`
is its consumer.

## Placement text for the app (DanceAround)

Phone centred in front of the play area, rear camera towards the player, landscape, about 1 m
up; the player stands about 2.3 m away (1.8–2.8 m), within 0.9 m of the centre line.
