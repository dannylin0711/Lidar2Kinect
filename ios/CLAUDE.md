# Working in ios/ (the Apple side)

The root `CLAUDE.md` rules apply. This file says what the app's stream now feeds on Windows and
what the Windows side asks of the app next. Written from the Windows side at Danny's request
(2026-11-26, after rebasing onto app 0.9.4); keep it current from here on.

## Two games now read the stream

| Game | Windows receiver | Streams it asks for | State |
|---|---|---|---|
| Dance Evolution Arcade (x86) | `windows/kinect10`, `windows/spicehook` | skeleton, depth, colour | working in the game (fake sender, and Danny's own play with the app) |
| DanceAround (x64) | `windows/vp4u` — replaces the game's `VisionPoseWrapper.dll` | skeleton only | **playable with `l2k_fake_sender`**: the game's player calibration measures the figure mid play area, two songs played to the result screen at 30 pose frames/s; **not yet run with the app** |

Since app 0.10.0 (2026-11-26) the app and `shared/` have DanceAround's side of the stream: consumer kind 5,
CONFIG `rate`, the 30-joint BODY stream (`docs/02` §3, §4b; `docs/03` §4, §4b). **The Windows side has not
adopted them yet**: vp4u still sends `consumer_kind` 0 and asks for SKELETON at the default rate, so it gets
what it got before (SKELETON at 30, Dance Evolution's play area). It advertises `_l2k._udp`.
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

### The app's play-area options and DanceAround

The app takes the game from the receiver's `consumer_kind` (`L2KGame`, `l2k_game_of_consumer`; kept while no
receiver is connected; `-l2kGame 1` starts on DanceAround):

- **"Fit me into the game's play area"** (on by default): DanceAround's box is 1.80–2.80 m / ±0.90 m
  (the fit keeps 10 cm inside), Dance Evolution's 1.80–3.20 m / ±1.25 m.
- **"Best position for <game>"** (off by default): level frame, one player centred at 2.3 m for
  DanceAround (one player only), 2.5 m for Dance Evolution (1 or 2 players).

Both assume vp4u's default phone placement (`phone_x` 0.55, `phone_z` 0); moving it in `l2k.ini`
moves DanceAround's box without the app knowing.

## Done on the Apple side (0.10.0), waiting for the Windows side

- `L2K_CONSUMER_VP4U` = 5 → DanceAround's play area and standing point (ask 2 below).
- CONFIG `rate` (byte 43): the app sends up to that many skeleton / body frames a second; images
  stay at 30. 60 from the camera-only sources needs "Faster camera" on (the app says so) (ask 3).
- BODY (0x13, stream bit 0x8): 30 joints in vp4u's order, nose / eyes / ears from Vision 2D or
  ARKit, the rest placed with the formulas that were `vp4u_map.cpp`'s, now in `shared/`
  (`l2k_mapper_body_frame`, `l2k_body_frame_from_skeleton`) (ask 4).

## Next, in order

1. **Run the app against DanceAround.** The Windows side is installed on Danny's PC (spice2x,
   `F:\BEMANI\UDN-001-2022062801`); the app finds it by Bonjour or the PC's address, UDP 24810.
   Stand about 2.3 m from the phone. Watch: the game's log `l2k: sender "…" connected`, its
   player calibration (`GamePlayerCalib:result pos (x, y, z)` should be near (0.55, −0.2, 2.3)),
   and **left/right**: raise the right arm, the avatar should mirror it like a mirror would (if
   not, `[vp4u] mirror_x=1` on Windows — tell the Windows side, it is open point 1 there).
2. ~~A `consumer_kind` for the vp4u receiver~~ — done in `shared/` and the app (0.10.0); the
   Windows side sends it next.
3. ~~60 fps for DanceAround~~ — CONFIG `rate`, done in `shared/` and the app; the Windows side asks
   for 60 next. To see on a device: how many frames a second Vision keeps up with at 60
   (the "Camera frames … used" diagnostics line; the receiver card's "Frames a second").
4. ~~A 30-joint body stream~~ — BODY, done in `shared/` and the app; the Windows side asks for it
   and maps it next (`windows/AGENT_PROMPT.md`, "DanceAround: what the Apple side added").
5. **Hand tips and thumbs from the picture.** No source tracks them yet: they are placed from the
   wrist and hand. Vision's hand pose request (`VNDetectHumanHandPoseRequest`) would give them at a
   cost per frame; worth it only if DanceAround is found to use the hands.

## Placement text for the app (DanceAround)

Phone centred in front of the play area, rear camera towards the player, landscape, about 1 m
up; the player stands about 2.3 m away (1.8–2.8 m), within 0.9 m of the centre line. Shown in the
receiver card when the game is DanceAround (0.10.0).
