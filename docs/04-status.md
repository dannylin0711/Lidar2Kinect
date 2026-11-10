# Status

Newest first. Each side appends its own entries; the Windows side also keeps `windows/STATUS.md`.

## 2026-11-10 — Windows side working in the game (Windows side)

Details and measurements: `windows/STATUS.md`.

- `shared/` builds with MSVC x86 without changes or warnings; 732 checks pass, golden packets
  match; `l2k_dump` ↔ `l2k_fake_sender` run over loopback at 30 fps. **No change to `shared/`.**
- 2b `Kinect10.dll` (all 26 exports at the real ordinals) and 2a spice2x hook built on one core.
  Probe (game's call pattern, imports by ordinal) passes at 30 fps per stream, including
  disconnect / reconnect. In the game under spice2x, fed by `l2k_fake_sender`: no sensor error,
  SENSOR CHECK shows camera, silhouette and skeleton, a song was played; both 2b and 2a.
- docs/01: ordinal 9 corrected to `NuiImageGetColorPixelCoordinatesFromDepthPixel`; open points
  1 and 2 resolved (`l2k_smoother_apply` equals Microsoft's filter to float precision); recovery
  logic of the Nui thread, held frames and tracked-skeleton calls added; new open point 6 (the
  SENSOR CHECK skeleton panel sits higher than the silhouette).
- Not done: a run with the real iPhone app.
- **`shared/` change (synthetic scene only, no protocol change, golden packets unchanged):**
  `l2k_synth_body_at(t, player, players, height, tilt, out)` and `l2k_synth_up(tilt)` place the
  synthetic camera anywhere; `L2K_SYNTH_CABINET_HEIGHT` 2.0 m / `L2K_SYNTH_CABINET_TILT` −24° is the
  cabinet-like scene (Danny: the cabinet's Kinect is about 2 m up). `l2k_synth_body` and the
  `L2K_SYNTH_UP_*` / `L2K_SYNTH_CAMERA_HEIGHT` macros are unchanged, so `SyntheticSource` and the
  Swift tests are unaffected. `l2k_fake_sender` now uses the cabinet scene, `-H` / `-a` override it.
  New test `test_cabinet_scene` (782 checks pass on Windows). **Not run on macOS:** `make -C shared
  test` and `swift test` — please run them.

## 2026-11-10 (evening) — upper-body mode (Apple side)

- Danny asked for tracking of just the upper half of the body, with leg notes left to miss.
- **`shared/` addition:** `l2k_map_params.upper_body` (+ three ratio fields) and the frame flag
  `L2K_FRAME_UPPER_BODY` (flags bit 1; `docs/02` §4, `docs/03` §4). Existing packets and golden
  files are unchanged; a receiver that ignores the flag keeps working. `make -C shared test`: 902 checks.
- App 0.3.0: "Upper body only" switch (off by default, `-l2kUpperBody 1`).
- Not run on a device. Whether ARKit detects a body at all with the legs out of frame is unknown.
- Seen in passing: the app (in the simulator on the Mac) lists the PC's DNS-SD advertisement,
  "DESKTOP-77BJ5IQ Lidar2Kinect", under "Found on this network" — the Windows side's open point
  about the advertisement not having been seen from the app is answered for the simulator.

## 2026-11-10 (later) — first device run, LiDAR depth source (Apple side)

- Danny installed 0.1.0 on a device: the body-tracking source reported no depth. ARKit's
  documentation gives `sceneDepth` to the world-tracking configuration only.
- New source `ARKitDepthSource` (app: "LiDAR depth", the default where supported): world tracking
  with scene depth, person segmentation and 2D body detection; joints lifted with the depth map
  (`DepthLift`, `docs/03` §7). The lifting maths is tested on the Mac
  (`testDepthLiftRecoversTheBody`); **the source itself has not run on a device.**
- App 0.2.0: three-way source picker, a "This device" card listing what ARKit supports.
- **`shared/` behaviour change (mapper):** a joint the source does not give is now reported
  INFERRED at its last known offset from its parent, never NOT_TRACKED — Dance Evolution drops a
  player unless all 20 joint states are nonzero. `l2k_mapper` grew a `memory` field; the wire
  format and the golden packets are unchanged. `make -C shared test`: 809 checks. Windows side:
  rebuild against the new `shared/`; no code change expected.

## 2026-11-10 — project start (Apple side)

Done and verified on the Mac:

- Kinect surface of `gamekdm.dll` read from the DLL (`docs/01`).
- Shared C library: types, protocol L2K1, depth codec D1, mapper, virtual camera, smoother,
  synthetic figure. `make -C shared test`: 732 checks pass under ASan/UBSan. Golden packets written.
- Tools: `l2k_dump` (reference receiver), `l2k_fake_sender`. Loopback run: 30 fps of skeleton,
  depth and colour, no loss, colour file identical after reassembly.
- Swift package: `swift test` passes (golden packets decode, pipeline, sender ↔ receiver handshake).
- App: builds for simulator and device. In the iPad simulator with the test figure it streams all
  three streams at 30 fps to `l2k_dump` on the Mac.

Not verified:

- **Nothing has run on a real device.** `ARKitBodySource` compiles but has never seen a camera:
  joint names, depth and segmentation availability, frame rate and latency are all unchecked.
- The Windows build of `shared/` (CMake, Winsock branch of the tools) has not been compiled.
- Calibration against a real Kinect (`docs/03` §6) and the open points of `docs/01` §7.

Next on the Apple side:

1. Run the app on an iPhone/iPad with LiDAR; fix what the camera source gets wrong; measure latency.
2. Decide the second source (Vision + depth) once the ARKit numbers are known.
3. Start the game port's RE of how the skeleton is judged (`docs/01` §7 points 3–5).
