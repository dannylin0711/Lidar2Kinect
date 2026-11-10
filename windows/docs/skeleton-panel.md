# Sensor height and tilt as the game sees them (open)

Started from Danny's report (2026-11-10) that in SENSOR CHECK the SKELETON panel sits higher than
the 3D DEPTH SENSORS silhouette. Measured with `l2k_fake_sender` in four placements, screenshots
taken by Danny in the game (`contents\screenshots`).

## The panels

SENSOR CHECK shows three 120×90 panels: RGB CAMERA (`[$CameraImage]`), 3D DEPTH SENSORS
(`[$SensorImage]`) and SKELETON (`[$BoneImage]`) — the skeleton is its own panel, not an overlay
(labels in `arkkdm.dll`). The 3D DEPTH panel is the depth image as delivered.

## Measurements (figure 1.7 m, about 2.5 m from the sensor; fractions of the panel height from the top)

| Sensor height, tilt | SKELETON panel | 3D DEPTH panel |
|---|---|---|
| 1.0 m, 0° | hips 0.30, feet 0.71, shoulders 0.05, head cut off at the top | hips 0.55, head 0.20, feet 0.98 |
| 2.0 m, −22° | hips ≈ 0.75, legs cut off at the bottom | head ≈ 0.15, feet ≈ 0.87 |
| 2.0 m, −27° | **same as at −22°** | moved up (head ≈ 0.08) |
| 1.4 m, −16° | head 0.15, hips 0.47, feet 0.93 — whole body, centred | head at the top edge |

## What this shows

- **The SKELETON panel undoes the tilt.** Changing only the tilt (−22° → −27°) moved the
  silhouette but not the skeleton. On each skeleton frame the game turns the integer elevation
  angle (from `NuiCameraElevationGetAngle`) into its own up vector (`gamekdm.dll 0x100100d0`,
  stored at `hid::Natal + 0x20`); the panel uses the level-corrected joints.
- **Its vertical position depends on the sensor height only.** One model fits all four
  placements within about 2 % of the panel: perspective from a *level* sensor at the same
  height, scale = the SDK focal length (285.63 px at 240 rows), panel centre a quarter of the panel
  above the horizon. For a player 2.5 m away the panel is centred when the sensor is about
  **1.4 m** above the floor; from 2 m the skeleton sits low with the legs cut off, and a real
  Kinect at 2 m should look the same.
- **The 3D DEPTH panel is the camera's own view** and depends on the tilt. For a sensor 2 m up
  and a player 2.5 m away, −24° centres the figure; `l2k_fake_sender` now uses that scene by
  default (`L2K_SYNTH_CABINET_HEIGHT` / `L2K_SYNTH_CABINET_TILT`, `-H` / `-a`).
- Our joints and silhouette agree with each other under the SDK projection in every placement:
  nothing here is a defect of the DLL or of `shared/`.

## What the game asks for

- **SENSOR CHECK → P1 + P2 START ("RESET ELEVATION ANGLE")** writes 27 (`0x1B`) into the wanted
  elevation angle (`hid::Natal + 0x18`, code at `0x100050c4` / `0x100051a3`) and prints the current
  angle (`+ 0x1c`) as "ELEVATION ANGLE ADJUSTED (%d)". **Observed:** the game then calls
  `NuiCameraElevationSetAngle(27)` (DLL log, 2026-11-10). No other `SetAngle` call was seen in
  boot, attract mode, test mode or a song.
- **No sensor height** is stored in the game: no constant for it was found in `gamekdm.dll` or
  `arkkdm.dll`, and none in `prop\` / `dev\`. The floor comes from the Kinect's
  `vFloorClipPlane` (`hid::Natal + 0x2c`).

## Open

- **+27° is the motor's maximum upward tilt.** Danny: the cabinet's Kinect sits at the top, about
  2 m up, probably on an angled mount. **Most likely reading:** the Kinect v1 elevation angle is
  relative to gravity (SDK documentation, not checked here) while the motor moves only ±27° relative
  to its base; on a mount that already points the sensor down, +27° cannot be reached, the motor
  stops at its upper end, and the game reads back and prints the angle actually reached ("ELEVATION
  ANGLE ADJUSTED (%d)" prints the current angle, `+ 0x1c`, not the 27 it asked for). RESET would
  then mean "tilt as far up as the mount allows", the mount fixing the final angle. For the
  replacement this changes nothing: it reports the phone's real tilt, and ignoring `SetAngle`
  behaves like a motor already at its stop. Not confirmed: the mount angle of a real cabinet.
  Other readings ruled less likely: an upright, level-based Kinect at 2 m tilted up 27° would not
  see the player; an upside-down one would deliver upside-down images, which the game does not
  appear to flip (our upright silhouette is shown upright). To confirm: the rest of the reset logic
  (`0x10005063`…`0x10005234`, state at `0x103e5f20`) or a photo of a cabinet's Kinect mount.
- Whether the judgement depends on the sensor height the way the SKELETON panel does (docs/01
  §7 points 3–4, port RE).
