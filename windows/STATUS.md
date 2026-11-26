# Windows status

Newest first. Machine: Windows 11 Pro 26200, Visual Studio 2022 17.14 (MSVC 14.44, bundled CMake
3.31), Kinect for Windows Runtime 1.8 installed, no real Kinect. Game: Dance Evolution Arcade
KDM-2016021900 with spice2x 2026-10-30 in `F:\BEMANI\DEA\KDM-2016021900\contents`.
DanceAround UDN-2022062801 dump in `F:\BEMANI\UDN-001-2022062801`, a spice2x runner for it in
`F:\下載\runner-dar-spice-`; GPU RTX 4070 Ti.

## 2026-11-26 — DanceAround: vp4u wrapper, phase 1 (builds, tests pass, not run in the game)

Why: DanceAround's camera pipeline (two RealSense + NEXT-SYSTEM VisionPose on TensorRT 7.2.1)
cannot run on an RTX 40 card (TensorRT 7.2 has no sm_89) and needs a hardware-bound VisionPose
activation. Replacing `VisionPoseWrapper.dll`, the game's only native entry into that pipeline,
avoids all of it.

Done:

- **Facts about the game's interface**: `docs/vp4u-interface.md` (from the game's managed
  metadata/IL, the original DLL's exports, the configs, a log of the original wrapper). Function
  table, structs, callbacks, boot order, what must return what, camera image format, coordinates
  (with the inferred parts marked), open points.
- **Core split, no behaviour change**: `core/l2kw_base.h` holds config / log / receiver / hub
  interface without `nui_abi.h`; `l2kw.h` includes it and adds the Kinect parts. The receiver,
  util, JPEG and DNS-SD files include only the base header, so they also build for x64.
  `init_module` takes an optional product name for the log's first line.
- **`vp4u/`** (own CMake project, x64): `visionposewrapper.dll` (four exports at the original
  ordinals, the 29-entry table, a dispatcher thread for the callbacks), `vp4u_map` (Kinect v1 frame
  → 30-joint vp4u bodies: levelled by `up`, placed in the game's frame, 10 joints derived),
  `vp4u_probe.exe` (the game's call order), `test_vp4u.exe`. Config `[vp4u]` in `l2k.ini`,
  `vp4u/README.md`.

Verified:

| What | How |
|---|---|
| x86 side unchanged | `cmake --build build/windows` (no warnings), `ctest --test-dir build/windows`: 4/4 (`core`, `probe_stream`, `probe_reconnect`, `l2k`) |
| shared builds x64 | `test_l2k` x64 passes against the golden packets |
| mapping | `test_vp4u`: 206 checks — axes (player's right +x, y up, z towards the player), a 2 m / −24° phone levelled onto a 1 m level one within 5–8 cm, floor at −camera height, play area, phone height override / fallback, mirror, yaw, two players, position-only slots left out |
| the DLL as the game calls it | `vp4u_probe --check` (ctest `vp4u_probe`, 6 s): version 210 / wrong version refused / NULL table refused, all 29 entries, Initialize non-NULL, LoadConfig reads LeftCameraPose.Y, Init* return 1, cameras active, ~25–29 empty frames/s without a sender, then with `l2k_fake_sender` 30.0 pose frames/s, 100 % with a body, 100 % with SpineBase inside the game's play area, all 30 joints valid, timestamps monotonic and < 100 ms old, RGB24 image valid; preview 28 frames/s per side, nothing after Stop; calibration ends with status 3; offline analysis ends with a NULL image; no pose callback after StopRealtimeAnalysis |

In the game (first run, Danny, spice2x, RTX 4070 Ti, no sender; `docs/vp4u-interface.md` §8):
the loader loaded the DLL and called `vp4uPreboot`; VisionPose came up ready through the
wrapper; test mode's camera check and calibration screens worked. Then
**`CameraErrorUncalibrated 5-1503-1001`**: the game requires `dev/nvram/stereo_cam.yml` in its
VFS (an encrypted `nvram2` mount). Second build: writes a placeholder there at `LoadConfig` and at
calibration commit (`[vp4u] calibration_file`), reports whether the loader hooked its file
imports, and falls back to `dev/raw/log/l2k.log` when `l2k.log` cannot be opened next to the DLL
(it was not, in the first run). `vp4u_probe` checks the placeholder (348 checks pass).

Second run: the wrapper's file calls are not redirected into the VFS (0 imports hooked; the
placeholder became a plain file under `…\game\dev\nvram\`), so the game still reported the
camera uncalibrated. `GameFs.IsFileExistVfs` asks avs through KAMUNITY (`FsLstat`). Third build:
writes and checks the placeholder through `kamunity.dll`'s `FsOpen` / `FsWrite` / `FsClose` /
`FsLstat` exports (the game's own native bridge to avs). Builds, tests pass; the avs path itself
only runs inside the game.

Third run: **the game boots to its attract demo with the wrapper** — the placeholder is in the VFS
("written to the game's VFS"), no camera error, only the network-services error (no server).
Empty pose frames ~24.5/s throughout. `vp4uPreboot` gets the config's text, not a path; the log
line no longer prints it (fourth build, not yet installed).

Not done / not verified:

- A run with a sender (fake sender, then the iPhone app): player detection, left/right, origin. Open points 1–7 of
  `docs/vp4u-interface.md`, above all left/right and the origin.
- Not with the real iPhone app.
- No change to `shared/` or `ios/`. Phase 2 (30-joint stream, 60 fps, a `L2K_CONSUMER_*` value for
  vp4u — CONFIG now sends 0 "unknown") needs both sides.

## 2026-11-10 — first Windows build, 2b and 2a working in the game

### "Done" items of the brief

| # | Item | State | How it was verified |
|---|---|---|---|
| 1 | `shared` tests pass on Windows, golden packets match | **done** | First MSVC (x86) build: no warnings, nothing to fix. `test_l2k`: 732 checks pass, byte-for-byte against `shared/tests/vectors`. `l2k_dump` ↔ `l2k_fake_sender` over loopback: 30 fps of skeleton, depth (~10 kB D1) and colour, 0 lost, colour JPEG identical after reassembly. |
| 2 | Probe receives skeleton, depth, colour at 30 fps through the DLL; events and status as docs/01 requires | **done** | `ctest` `probe_stream` and `probe_reconnect` (below), run on every build. Measured 30.0–30.25 frames/s per stream, 0 wake-ups without a frame, colour identical to the JPEG, LockRect pitch 640 / 2560. |
| 3 | The game, under spice2x with the replacement DLL, runs a song from `l2k_fake_sender` with no sensor error | **done** | Danny watched: no sensor error; SENSOR CHECK shows the test card, the silhouette, the skeleton, STATUS READY; the figure moves; a song was played. The DLL's log: 150 skeleton / depth / colour frames taken per 5 s, smoothing on every frame, 0 "no data". |
| 4 | This file | **done** | |

### Built (`windows/`, see `README.md`)

- **core** — receiver thread (UDP 24810, HELLO → CONFIG with the streams the game has open, sent
  again at once when that changes; skeleton newest-wins; depth D1/raw and colour JPEG via WIC;
  3 s silence / BYE = gone; DNS-SD `_l2k._udp` through dnsapi.dll loaded at run time) and the
  virtual sensor (`INuiSensor` in SDK 1.8 order, frame textures held until released, manual-reset
  events set on a new frame and reset when taken, frame-end after each frame and at least every
  100 ms, `NuiStatus` + status callback from the sender's presence, elevation from `up`, set-angle
  accepted and ignored, ordinal 9 = ×2, ordinal 20 = `l2k_smoother_apply`, depth drawn from the
  skeleton when the sender sends none). Config `l2k.ini` and log `l2k.log` next to the DLL
  (`docs/config.md`).
- **2b `Kinect10.dll`** — all 26 exports of Microsoft's 1.8 DLL at the real ordinals, same stack
  sizes (checked by disassembling both), names included; ordinal 1 (unnamed, unknown purpose)
  returns a pointer to a zeroed block. Static CRT; imports only system DLLs (ws2_32,
  windowscodecs, ole32, oleaut32, kernel32).
- **2a `l2k_spicehook.dll`** — loaded with `-k`; rewrites `gamekdm.dll`'s Kinect10 import slots
  (by ordinal or name) to the core.
- **probe `l2k_probe.exe`** — imports the five functions by ordinal through an import library
  built from a NONAME `.def` (verified with `dumpbin /imports`: "Ordinal 5, 6, 9, 20, 22"), runs
  the game's thread logic (`0x1000ebd0` and callees): five events of the same types, the same
  `WaitForMultipleObjects`, the same status → state mapping, frames held until the next event,
  `NuiTransformSmooth` on every skeleton frame, ordinal 9 on the four corners.
- **tools** `l2k_smoothcmp.exe`; **tests** `test_core.exe` (sensor without network, 112 checks).

### Tests (`ctest --test-dir build/windows -C Release`): 4/4 pass

- `l2k` — shared golden packets.
- `core` — conversions, ordinal 9, missing → present (callback S_OK) → sensor, events set on a
  new frame and reset when taken, `E_NUI_FRAME_NO_DATA` otherwise, frame-end, depth drawn from the
  skeleton with player index = slot + 1, two frames held at once in different textures, colour
  copied exactly, elevation 10° from a tilted `up`, set-angle ignored, ordinal 20 identical to
  `l2k_smoother_apply`, sender gone → `E_NUI_NOTCONNECTED` + callback.
- `probe_stream` — 8 s, two players, test card: ≥ 27 fps per stream and frame-end, ≤ 3 empty
  wake-ups/s, 2 tracked every second, > 1000 player pixels all within 800–4000 mm, pitches,
  colour = JPEG decoded by WIC, ordinal 9 corners, state running.
- `probe_reconnect` — the same, then: BYE → state "sensor missing" in 0.00 s; sender back →
  running again in 0.02 s through the status callback; sender killed without BYE → missing after
  3.01 s.

### Resolved points of docs/01 §7

- **1, export ordinals** — confirmed 5, 6, 20, 22; **ordinal 9 corrected** to
  `NuiImageGetColorPixelCoordinatesFromDepthPixel` (7 args), docs/01 §2 fixed.
  `docs/kinect10-exports.md` has the full table.
- **2, smoothing** — `l2k_smoother_apply` equals Microsoft's filter to float precision
  (`docs/smoothing.md`). Microsoft's flat `NuiTransformSmooth` needs a sensor; the comparison
  calls the DLL's internal routine. `shared/` unchanged.
- Struct sizes/offsets and vtable offsets the game uses: `static_assert`ed in `core/nui_abi.h`
  (from the documented ABI; the SDK headers were not installed).
- New facts added to docs/01: the status callback / retry-event recovery logic, frames held until
  the next event, tracking flag 2 and the tracked-skeleton calls seen in the game, the game's own
  up vector from the elevation angle.

### Questions of the brief

- **Which Kinect10.dll the game loads** — the one in `contents\` (next to `spice.exe`) wins over
  SysWOW64: read in spice2x's source and observed (`docs/spice2x.md`).
- **Does spice2x do anything about this game's Kinect** — no: it loads `gamekdm.dll` itself and
  only checks that a `Kinect10.dll` can be loaded (`docs/spice2x.md`).
- **No sender** — both behaviours implemented and tried in the game (`docs/no-sender.md`).
  Default `missing`: the game shows a sensor error while no sender is connected and clears it by
  itself when one connects (observed mid-game). `idle`: no error, an empty view (observed at boot).
  Not observed: `missing` with no sender during the whole boot.
- **What 2a adds over 2b on Danny's setup** — little. Verified in the game: with Microsoft's
  runtime installed and `spice.exe -w -ea -k l2k_spicehook.dll`, the hook redirected all five
  imports (`#5 #6 #20 #9 #22`) before the game's first call and the game ran on the core exactly
  as with 2b. What it adds: the Microsoft runtime can stay installed and the game folder keeps no
  file named `Kinect10.dll`; switching is a spice2x option (`-k`) instead of a file rename. What
  it needs: *some* `Kinect10.dll` for `gamekdm.dll` to load at all (Microsoft's runtime, or 2b).
  2b alone is the simpler install and the recommended one.

### Danny's game folder now

`contents\` holds `l2k_spicehook.dll`, `l2k.ini` (default settings), `Kinect10.dll.l2k` (2b,
renamed aside for the 2a test — rename back to `Kinect10.dll` to use 2b), and `test.bat` =
`spice.exe -w -ea -k l2k_spicehook.dll` (Danny's options kept, hook added). Also created the
missing `dev\raw\log` folder: without it AVS could not create its log and the game crashed at boot
before loading `gamekdm.dll` (unrelated to the Kinect). Logs of the runs: `l2k-run2-2b.log`,
`log-run1-avs-crash.txt`, `log-run2-2b.txt`.

### Sensor placement (later the same day)

- The fake scene now matches the cabinet (Danny: Kinect about 2 m up): `l2k_fake_sender` films
  from 2 m, tilted −24° (`shared/` change, see `docs/04-status.md`; `-H` / `-a` options).
- Measured in four placements with Danny's screenshots: the SENSOR CHECK **SKELETON panel removes
  the tilt and depends only on the sensor height** (−22° → −27° did not move it); a 2.5 m player is
  centred there with the sensor ≈ 1.4 m up. The 3D DEPTH panel is the camera view and depends on
  the tilt; −24° centres it at 2 m.
- RESET ELEVATION ANGLE (P1 + P2 START) makes the game call `NuiCameraElevationSetAngle(27)` —
  observed in the log. The game holds no sensor height.
- Details and the open questions: `docs/skeleton-panel.md`.

### Open

- **Sensor placement:** the game asks for +27° (the maximum upward tilt) although the cabinet's
  Kinect is about 2 m up. Most likely a tilted mount (Danny): RESET lifts the sensor 27° above the
  mount's direction and the game reads back the angle reached — nothing to change in the DLL, but
  the cabinet's mount angle is not confirmed. Also open: whether the judgement depends on sensor height like the
  SKELETON panel does (`docs/skeleton-panel.md`).
- Not run with the real iPhone app yet (needs Danny): camera-source data, Wi-Fi rates, latency.
- `NuiSkeletonSetTrackedSkeletons` is passed through, not enforced (the sender tracks ≤ 2 bodies;
  the game picks the tracked one's id, so the SDK's behaviour would give the same frames).
- The DNS-SD advertisement registers (`registration done (0)` in the log) but has not been seen
  from the app yet.
- Behaviour of the real runtime that cannot be seen without a sensor (exact frame timestamps,
  the real ordinal-9 corner offsets, `NuiStatus` codes of an unplugged Kinect) — the replacement
  follows the SDK documentation there.
- Methods the game never calls are laid out from the SDK documentation only; the SDK headers were
  not available to cross-check them.
