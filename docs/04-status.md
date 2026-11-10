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
