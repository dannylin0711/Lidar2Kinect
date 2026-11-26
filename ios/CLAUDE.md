# Working in ios/ (the Apple side)

The root `CLAUDE.md` rules apply. This file says what the app's stream now feeds on Windows and
what the Windows side asks of the app next. Written from the Windows side at Danny's request
(2026-11-26, after rebasing onto app 0.9.4); keep it current from here on.

## Two games now read the stream

| Game | Windows receiver | Streams it asks for | State |
|---|---|---|---|
| Dance Evolution Arcade (x86) | `windows/kinect10`, `windows/spicehook` | skeleton, depth, colour | working in the game (fake sender, and Danny's own play with the app) |
| DanceAround (x64) | `windows/vp4u` — replaces the game's `VisionPoseWrapper.dll` | skeleton + body, 60 a second | **playable with `l2k_fake_sender`**: the game's player calibration measures the figure mid play area, two songs played to the result screen; **not yet run with the app** |

Both sides now have DanceAround's part of the stream (app 0.10.0, Windows 2026-11-26): vp4u sends
`consumer_kind` 5 (`L2K_CONSUMER_VP4U`) and CONFIG `rate` 60, and asks for SKELETON + BODY (`docs/02` §3,
§4b; `docs/03` §4, §4b). It advertises `_l2k._udp`. Details: `windows/vp4u/README.md`,
`windows/docs/vp4u-interface.md`, `docs/04-status.md`.

## What DanceAround relies on — do not break

`windows/vp4u` turns each BODY frame (or, without one, its SKELETON frame through
`l2k_body_frame_from_skeleton`) into the game's 30-joint bodies (`vp4u_map.cpp`):

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

## Done on both sides (app 0.10.0; Windows 2026-11-26)

- `L2K_CONSUMER_VP4U` = 5 → DanceAround's play area and standing point (ask 2 below). vp4u sends it.
- CONFIG `rate` (byte 43): the app sends up to that many skeleton / body frames a second; images
  stay at 30. 60 from the camera-only sources needs "Faster camera" on (the app says so) (ask 3).
  vp4u asks for 60 (`[vp4u] rate`); Dance Evolution's receivers ask for 0 (30).
- BODY (0x13, stream bit 0x8): 30 joints in vp4u's order, nose / eyes / ears from Vision 2D or
  ARKit, the rest placed with the formulas that were `vp4u_map.cpp`'s, now in `shared/`
  (`l2k_mapper_body_frame`, `l2k_body_frame_from_skeleton`) (ask 4). vp4u uses a frame's BODY, and
  its SKELETON through `l2k_body_frame_from_skeleton` when the sender offers no BODY or the BODY
  datagram was lost (8 ms wait); `[vp4u] body=0` asks for SKELETON only.
- Checked on Windows with `l2k_fake_sender` (`vp4u_probe`): 60.0 pose frames a second from BODY,
  all 30 joints valid, every frame inside the play area; and 60.0 from SKELETON with `body=0`.

## Next, in order

1. **Run the app against DanceAround.** The Windows side is installed on Danny's PC (spice2x,
   `F:\BEMANI\UDN-001-2022062801`); the app finds it by Bonjour or the PC's address, UDP 24810.
   Stand about 2.3 m from the phone. Watch: the game's log `l2k: sender "…" connected`, its
   player calibration (`GamePlayerCalib:result pos (x, y, z)` should be near (0.55, −0.2, 2.3)),
   and **left/right**: raise the right arm, the avatar should mirror it like a mirror would (if
   not, `[vp4u] mirror_x=1` on Windows — tell the Windows side, it is open point 1 there).
2. ~~A `consumer_kind` for the vp4u receiver~~ — done on both sides.
3. ~~60 fps for DanceAround~~ — done on both sides. To see on a device: how many frames a second
   Vision keeps up with at 60 (the "Camera frames … used" diagnostics line; the receiver card's
   "Frames a second"); the game log's `l2k.log` "pose frames N/s (… from BODY …)" shows what arrives.
4. ~~A 30-joint body stream~~ — done on both sides. On a device: the game log says
   `l2k: pose frames from the sender's BODY stream (30 joints)` once the BODY frames are used.
5. **Hand tips and thumbs from the picture.** No source tracks them yet: they are placed from the
   wrist and hand. Vision's hand pose request (`VNDetectHumanHandPoseRequest`) would give them at a
   cost per frame; worth it only if DanceAround is found to use the hands.

## Placement text for the app (DanceAround)

Phone centred in front of the play area, rear camera towards the player, landscape, about 1 m
up; the player stands about 2.3 m away (1.8–2.8 m), within 0.9 m of the centre line. Shown in the
receiver card when the game is DanceAround (0.10.0).
