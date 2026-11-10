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

## 2026-11-10 (late night, 5) — why a player cannot come back; play-area fit; ultra-wide depths (Apple side)

- Danny, playing the real game with the app: (1) the left hand is wrong in the game though right
  in the app's picture; (2) after a detection failure mid-game the player cannot reconnect.
- (2) is the game's own rule, now read from the DLL and run under the emulator (`docs/01` §7): a
  body is a player candidate only 1.80–3.20 m from the sensor, ±1.25 m sideways, hip 0–2 m above
  the reported floor; 5 cm slack to enter, 20 cm to stay. The tracking id is not what blocks a
  return. In ultra-wide mode Danny stood about 1.2–1.6 m away, and that mode's floor estimate was
  unstable.
- **`shared/` addition:** `fit_play_area` (+ area parameters) in `l2k_map_params`: slides a tracked
  body outside the box to just inside, keeps the floor consistent. Off by default in the library,
  on by default in the app ("Fit me into the game's play area"). 1033 checks.
- (1): the app's picture only shows the 2D projection; in ultra-wide mode the depths came from the
  3D model, which does not fit the picture well. Depths are now solved from the picture and bone
  lengths (`VisionKinematics`, `docs/03` §7). Also new: a top-down inset in the preview so depth
  errors can be seen. **Not yet checked on a person.** If Danny plays in LiDAR mode instead, this
  change does not apply and (1) is still unexplained.
- App 0.7.0.

## 2026-11-10 (late night, 4) — colour stream can be switched off (Apple side)

- Danny asked for an option to disable the RGB camera feed. App 0.6.2: "Send the camera picture"
  (Receiver card, on by default, `-l2kSendColor 0`). Off: HELLO no longer offers the colour stream
  and no colour packet is sent even if the receiver asks (`testColourCanBeSwitchedOff`). The
  skeleton and the depth image are unaffected; the on-device preview still shows the camera.
- Windows side: with colour off the game's colour texture gets no frames — the DLL should keep
  showing black (or its last frame) and must not treat that as a sensor error.

## 2026-11-10 (late night, 3) — hands lost at times; measuring before changing more (Apple side)

- Danny: the left hand is still missed sometimes. Not yet reproduced or measured on the device.
- What a picture already shows (Vision 2D on a frame of five small dancers): wrist confidences sit
  around the 0.2 cut-off (0.14–0.58), so wrists of a small or blurred figure are dropped often.
- App 0.6.1, low-risk changes: exposures limited to 1/120 s in both camera sources (less motion blur);
  in the ultra-wide source a joint the picture does not show keeps the 3D model's offset from its
  parent, hung on the parent as placed (an unseen wrist follows the seen elbow); a diagnostics line
  with how often each wrist and ankle was seen over the last 60 frames.
- **"Record 10 s"** (diagnostics card, or `-l2kRecord 10`): saves the camera to
  `Documents/l2k_record.mov` + `.json`. Plan: pull a clip of Danny moving, run the detectors over
  it on the Mac, and decide from the numbers between cropping to the person, a lower cut-off, and
  temporal filtering. None of those is implemented yet.

## 2026-11-10 (late night, 2) — players kept through dropouts (Apple side)

- Danny: when the game loses the player and the player comes back, the game does not see the same
  person and will not go on. Cause on our side: any missed detection gave the body a new tracking id.
- **`shared/` behaviour change (mapper):** tracking ids are now the mapper's own (1, 2, …); a lost
  body is held for 0.5 s with its last skeleton (joints inferred); a returning body takes back the
  nearest remembered slot and id (`keep_identity`, on by default). `l2k_mapper` and
  `l2k_map_params` grew fields; the wire format and golden packets are unchanged.
  `make -C shared test`: 941 checks. Windows side: rebuild; no code change expected.
- Ultra-wide source: joint positions now come from Vision's 2D pose and distances from its 3D pose,
  after device snapshots showed the 3D request's own image positions sitting off the body. Checked
  on a picture on the Mac; not yet seen on a person on the device.
- App 0.6.0: "Keep the same player" switch.
- Open: what the game itself does when its tracked id disappears (to be read in the port's RE).

## 2026-11-10 (late night) — first runs on the iPhone, ultra-wide source (Apple side)

- Signed builds now work from the command line (`xcodebuild -allowProvisioningUpdates`, a profile
  for `dev.latowolf.lidar2kinect` exists). The app is installed on Danny's iPhone 17 Pro; its
  diagnostics are read with `devicectl device process launch --console … -- <bundle> -l2kLog 1`.
- LiDAR source on the device: depth map present once camera tracking is normal, person mask from
  ARKit, 30 processed frames/s. No person was in view, so the lifted skeleton is still unseen.
- New `UltraWideSource` (app: "Ultra-wide"): rear ultra-wide camera + Vision 3D pose, no LiDAR
  (`docs/03` §7). On the device: 30 fps, a body tracked. Its accuracy has not been assessed.
- App 0.5.0: the ultra-wide option; a colour preview zoomed out to twice the Kinect's view for it;
  `-l2kLog 1` console logging; `-l2kVisionMask 1`. Fixed in passing: the preview flashed, because
  frames without a picture (between the 15-per-second preview frames) blanked it.
- Vision's left/right confirmed as the person's own on a picture of dancers facing the viewer.

## 2026-11-10 (night) — device facts, LiDAR depth source rebuilt on Vision (Apple side)

- Danny ran 0.3.0 on an iPad Pro (M2) and an iPhone 17 Pro: no skeleton, no depth. The "This
  device" card on both: 3D body tracking yes; world tracking + LiDAR depth yes; every combination
  of the depth map with a body (2D or 3D) no. So `ARKitDepthSource` (ARKit 2D body + depth) could
  never start on these devices; it is removed.
- New `VisionDepthSource` (app: "LiDAR depth"): ARKit world tracking for the depth map, Vision for
  the 2D body pose of up to two people, lifted with the depth map (`docs/03` §7). Vision's
  orientation handling is tested on the Mac with a QR code in all four device rolls.
- App 0.4.0: live diagnostics card (frame counts, depth map size / centre distance / range, person
  mask coverage, people seen, joints placed, errors) with a "Copy diagnostics" button; the depth
  preview now shows the raw depth map (before the Kinect range and players-only filters, which made
  it black when no person was in view); the skeleton's left side is orange and right side blue.
- ARKit's 2D and 3D joint names used by the sources were checked: the 2D list printed from the
  simulator, the 3D names against Apple's motion-capture documentation. All present.
- Not established: why the 3D body-tracking source showed no skeleton on the devices.
- **Nothing in this entry has run on a device yet.**
- Simulator test (new `L2K iPhone 17 Pro` simulator, synthetic figure → `l2k_dump` on the Mac): two
  players with the colour preview, and one player in upper-body mode with the depth preview, both a
  steady 30 fps on all three streams with nothing lost — once the Mac was idle. Under heavy load the
  rate collapsed, which exposed that the preview picture was built on the capture queue. App 0.4.1:
  the preview is built on its own queue and preview-only images are made 15 times a second.

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
