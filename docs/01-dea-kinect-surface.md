# What Dance Evolution Arcade asks of the Kinect

Source: static analysis of `gamekdm.dll` from the dump `KDM-2016021900` (PE32, x86, image base
`0x10000000`, SHA-256 `c763b0ef…d41a7a7`), Ghidra decompile kept in the game port's repository
(`~/code/danceevolution-ipad/re/export`). Addresses below are virtual addresses in that DLL.

Each statement is marked:

- **[DLL]** read directly from the game's code or import table.
- **[SDK]** from the Kinect for Windows SDK v1.x headers as remembered, matched to the game's
  call shapes (argument counts, constants, struct sizes). Confirm against the real `NuiApi.h` /
  `NuiSensor.h` before relying on an exact value.
- **[open]** not established yet.

## 1. Summary

The game uses the **Kinect for Windows SDK v1** (`Kinect10.dll`), one sensor, and three things from it:

| Stream | Request | Use in the game |
|---|---|---|
| Skeleton | tracking enabled, optional "title chooses the tracked skeletons" | the dance judgement input (`hid::Natal::copySkeleton`) |
| Depth + player index | 320×240 | uploaded to a texture named `NatalDepth` (D3D format L16) |
| Colour | 640×480 | uploaded to a texture named `NatalColor` (D3D format X8R8G8B8) |

It also reads and sets the sensor's tilt angle, polls the sensor status, and asks for the
depth→colour pixel mapping at the four corners of the depth image.

Nothing else of the SDK is touched: no audio, no coordinate mapper object, no near mode, no
seated mode, no face tracking.

## 2. Imports from Kinect10.dll

**[DLL]** `gamekdm.dll` imports five functions from `Kinect10.dll`, **by ordinal only** (no names):
5, 6, 9, 20, 22. No other game DLL imports the Kinect.

| Ordinal | IAT slot | Call site | Arguments seen | Function **[SDK]** |
|---|---|---|---|---|
| 5 | `0x102181d4` | `0x1000ece1` | `(int *count)`; result `< 0` = error, `count == 0` = no sensor | `NuiGetSensorCount` |
| 6 | `0x102181d8` | `0x1000ed27` | `(0, INuiSensor **out)` | `NuiCreateSensorByIndex` |
| 9 | `0x102181e0` | `0x10019cf5` in `FUN_10019c30` | `(2, NULL, x, y, depth_mm << 3, long *cx, long *cy)` — 7 arguments | `NuiImageGetColorPixelCoordinatesFromDepthPixel` (corrected 2026-11-10; was given as `…AtResolution`, which is ordinal 10 with 8 arguments) |
| 20 | — | in `FUN_100100d0`, once per new skeleton frame | `(NUI_SKELETON_FRAME *, params *)`, params = 0.5, 0.5, 0.5, 0.05, 0.04 | `NuiTransformSmooth` |
| 22 | `0x102181e4` | `0x1000ebe3` in `FUN_1000ebd0` | `(callback, NULL)` at thread start, `(NULL, NULL)` at exit | `NuiSetDeviceStatusCallback` |

**Confirmed 2026-11-10** against the export table of Microsoft's `Kinect10.dll` 1.8.0.595
(`windows/docs/kinect10-exports.md`, which lists all 26 exports with their stack sizes): 5, 6, 20
and 22 are as above; 9 is `NuiImageGetColorPixelCoordinatesFromDepthPixel` — the 7 arguments
pushed at `0x10019cf5` match its 28-byte stack, not the 8-argument `…AtResolution` (ordinal 10).
The import table lists them in the order 5, 6, 20, 9, 22. A replacement DLL must export these five
at exactly these ordinals.

## 3. The sensor object (INuiSensor)

The game keeps the `INuiSensor *` at `thread + 0x15fc` and calls it through the vtable. Offsets
are for the 32-bit vtable.

| Offset | Method **[SDK]** | How the game calls it **[DLL]** |
|---|---|---|
| `0x08` | `Release` | after `NuiShutdown`, at thread exit |
| `0x0c` | `NuiInitialize(flags)` | `flags = 0x0B` = depth-and-player-index \| colour \| skeleton (`0x1000ed3c`) |
| `0x10` | `NuiShutdown()` | at thread exit |
| `0x14` | `NuiSetFrameEndEvent(event, 0)` | once after initialise; the event is auto-reset |
| `0x18` | `NuiImageStreamOpen(type, resolution, flags, frameLimit, event, &handle)` | stream 0: `(0, 1, 0, 3, ev1, &h0)` = depth+player index, 320×240; stream 1: `(1, 2, 0, 3, ev2, &h1)` = colour, 640×480 |
| `0x24` | `NuiImageStreamGetNextFrame(handle, 0, &frame)` | when the stream's event fires; wait 0 ms |
| `0x28` | `NuiImageStreamReleaseFrame(handle, &frame)` | before fetching the next frame, and at exit |
| `0x38` | `NuiCameraElevationSetAngle(angle)` | when the wanted angle changes (`FUN_1000f510`) |
| `0x3c` | `NuiCameraElevationGetAngle(&angle)` | every wake-up; smoothed as `(new + 2·old) / 3` |
| `0x40` | `NuiSkeletonTrackingEnable(event, flags)` | `flags` = 0, or 2 (title sets tracked skeletons) when the game picks players |
| `0x44` | `NuiSkeletonTrackingDisable()` | before re-enabling with other flags, and when tracking is switched off |
| `0x48` | `NuiSkeletonSetTrackedSkeletons(ids[2])` | after each skeleton frame while flag 2 is on |
| `0x4c` | `NuiSkeletonGetNextFrame(0, &frame)` | when the skeleton event fires; frame is `0xA68` = 2664 bytes |
| `0x68` | `NuiStatus()` | polled in the thread loop, see §5 |

A replacement must lay the vtable out in the SDK's exact order, including the methods the game
never calls (they can return `E_NOTIMPL`).

**Image frames are held** **[DLL, `0x1000f400`]**: on a stream event the game first releases the
frame it holds, then takes the next one and keeps it (texture locked and copied on its own wake
event) until the next event of that stream. A replacement must keep a frame's buffer valid until
it is released.

**Seen in the running game (2026-11-10, replacement DLL log):** tracking is enabled with flag 2
(title sets the tracked skeletons), and `NuiSkeletonSetTrackedSkeletons` is called after each
skeleton frame with the ids `(0, id)` / `(id, 0)`, alternating every 2 s, where `id` is the
tracked body's tracking id. In SENSOR CHECK and while playing, ordinal 9 is called for the four
corners on every rendered frame (≈ 960 calls/s).

**Image frames.** `NUI_IMAGE_FRAME` is 40 bytes **[DLL: copied as 5×8 bytes]**. The game reads
`dwFrameNumber` at `+0x08` (a frame is uploaded only when this number changed) and
`pFrameTexture` at `+0x14`, then calls on the `INuiFrameTexture`:

- `+0x14` `LockRect(0, &lockedRect, NULL, 0)` and reads the pitch and the bits pointer,
- `+0x1c` `UnlockRect(0)`.

It copies `width·2` bytes per row for depth (16-bit) and `width·4` for colour (32-bit) — so
colour must be 32-bit BGRX, 640×480, and depth 16-bit packed (depth in mm `<< 3` \| player index)
**[SDK for the packing]**.

**Skeleton frames.** `FUN_100100d0` runs on each new frame: it reads the frame's
`vFloorClipPlane` (frame `+0x10`), then smooths the frame with ordinal 20 (always; an option bit
only chooses whether the unsmoothed copy is kept too). `FUN_100103d0` then walks all 6
`SkeletonData` slots. For a slot whose
`eTrackingState == 2` (tracked) it takes the 20 joint positions and the 20 per-joint tracking
states; for any other state the joint states are treated as 0. When an option bit is set it
rotates every joint by a quaternion first (tilt compensation is the likely purpose — **[open]**,
to be read in the port's RE).

## 4. Threading and events **[DLL]**

A dedicated thread (`hid::Natal::NuiThread`, body `FUN_1000ebd0`) owns the sensor. It waits with
`WaitForMultipleObjects(5 handles, FALSE, 30000)` on:

| Index | Event | Type | Meaning |
|---|---|---|---|
| 0 | wake | auto-reset | set by the game's main thread every frame: apply settings (tilt, tracking flags) |
| 1 | stream 0 | manual-reset | passed to `NuiImageStreamOpen` for depth |
| 2 | stream 1 | manual-reset | passed to `NuiImageStreamOpen` for colour |
| 3 | skeleton | manual-reset | passed to `NuiSkeletonTrackingEnable` |
| 4 | frame end | auto-reset | passed to `NuiSetFrameEndEvent` |

Consequences for a replacement:

- The stream and skeleton events are **manual-reset and the game does not reset them** on the
  normal path. The real SDK resets an event when its frame is taken. A replacement must reset
  the event inside `…GetNextFrame`, or the thread spins.
- When there is no new frame, `…GetNextFrame` must fail (the SDK returns `E_NUI_FRAME_NO_DATA`);
  the game tolerates any failure except the two "not connected / not ready" codes in §5.
- If **no frame-end event arrives for 30 s** the thread declares a timeout (state 6) and the game
  shows a sensor error. Signal the frame-end event once per frame.

## 5. Status handling **[DLL]**

Thread state at `thread + 0x28`: 1 = opening, 7 = running, 2 = stopping, 3 = error, 4 = sensor
missing, 5 = sensor not ready / not powered, 6 = timeout. `NuiStatus()` results map as:

| HRESULT | State |
|---|---|
| `>= 0` | keep running |
| `0x80070015` (`ERROR_NOT_READY`) | 5 |
| `0x8301027F` (NUI facility, insufficient power) | 5 |
| `0x8007048F` (`ERROR_DEVICE_NOT_CONNECTED`), `0x83010014`, `0x83010015` | 4 |
| anything else negative | 3 |

`NuiSkeletonGetNextFrame` / `NuiImageStreamGetNextFrame` returning `0x80070015` or `0x8007048F`
also sets state 4. `NuiGetSensorCount` failing gives 3, a count of 0 gives 4.

**Recovery [DLL]:** the thread sets a global auto-reset event once at start and waits on it before
every attempt to open the sensor; after any failure (states 3–6) it shuts the sensor down, releases
it and waits on that event again. The device status callback (`0x1000ebb0`, `(hr, instance,
unique, user)`, stdcall) does nothing but `SetEvent` on it, and only when `hr == S_OK` exactly. So
a sensor that appears later is picked up only if the runtime calls the callback with `S_OK`.
Observed with the replacement DLL: the game shows a sensor error while the sender is gone and
clears it by itself after the callback (`windows/docs/no-sender.md`).

**Elevation [DLL, `0x100100d0`]:** on each skeleton frame the game builds its own up vector from
the integer elevation angle (stored at `natal + 0x20`) rather than using `vNormalToGravity`, and
keeps the floor plane at `natal + 0x2c`.

The test-mode text `[$KinectStatus]` (in `gamekdm.dll` and `arkkdm.dll`) shows this state.

## 6. Depth → colour registration **[DLL]**

`FUN_10019c30` takes a distance clamped to 0.8…4.0 m and calls ordinal 9 for the depth pixels
(0,0), (320,0), (0,240), (320,240) with colour resolution 640×480. It divides the answers by 640
and 480 to get four texture coordinates; on failure it falls back to the unit square. So the game
draws the colour image through a quad that lines it up with the depth image.

In the virtual Kinect the two images come from the same camera, so the mapping is exact:
`colourX = 2·depthX`, `colourY = 2·depthY`, at every depth.

## 7. Open points

1. ~~Export ordinals of the real `Kinect10.dll`~~ — **resolved 2026-11-10** (§2; ordinal 9 corrected).
2. ~~Whether `l2k_smoother_apply` matches the real DLL's `NuiTransformSmooth`~~ — **resolved
   2026-11-10: it does**, to float precision, on 5 parameter sets with noise and jumps
   (`windows/docs/smoothing.md`).
3. What the game does with the depth and colour textures on screen, and whether it needs depth
   outside the player pixels. — port RE (shaders `gesture_depth`, `NatalCameraObjectRender…`).
4. The quaternion applied to the joints (tilt compensation?) and how the judgement uses joint
   states. — port RE of `FUN_100103d0` and its callers.
5. A second mode in `hid::Natal::Update` (`FUN_10012cb0`) replaces live skeletons with data read
   from a buffer of `0x140`-byte records — probably recorded motion for demos. — port RE.
6. Sensor placement. The SENSOR CHECK skeleton panel removes the tilt (elevation angle) and its
   position depends on the sensor height only; a 2.5 m player is centred with the sensor ≈ 1.4 m up
   (measured in four placements). RESET ELEVATION ANGLE asks for `NuiCameraElevationSetAngle(27)`,
   the maximum *upward* tilt (observed), while the cabinet's Kinect is about 2 m up (Danny),
   probably on an angled mount: most likely the motor stops at its upper end and the game prints
   the angle reached. The mount angle is not confirmed. —
   `windows/docs/skeleton-panel.md`.
