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

## 2026-11-11 (5) — 0.9.2: the preview jumping, the game's images on the player (Apple side)

Danny sent a screen recording from the iPhone 17 Pro (front camera, landscape, "Best position" on,
a receiver connected): the preview jumped between the whole camera picture and a close-up several
times a second, and "the game is not reading a full view of the camera". **No change to `shared/`.**

- **Jumping preview.** While a receiver takes the colour picture every frame carries the Kinect
  colour image, but only every second one the whole-camera picture made for the preview; the
  preview showed whichever came. Frames are now marked (`L2KFrame.forLocal`) and the preview only
  shows the ones made for it.
- **The game's images.** With the skeleton normalised to 2.5 m and the player standing close to a
  wide camera, the images sent were still the middle of the camera picture at the Kinect's field of
  view: a face and chest. They are now framed on the skeleton that is sent (`L2KGameView`, docs/03
  "Where the images look"); the preview's dashed rectangle shows that part, and the skeleton drawn
  over a picture made for the game is the sent one.
- The recording also shows 0.9.1's front camera working in landscape: picture upright, skeleton on
  the body from head to feet.
- Verified: `swift test` 19 tests (1 new: the fit, the camera that renders it for the four rolls,
  and through the pipeline a player at 1.2 m normalised to 2.5 m whose mask lies on the sent
  skeleton's joints, view scale 2.05); `make -C shared test` 1136 checks; the app builds and is
  installed on the iPhone and the iPad. **Not verified on a device**: both fixes by eye (Danny to
  confirm), the picture a receiver gets, two players.

## 2026-11-11 (4) — 0.9.1: front camera crash, picture a quarter turn off, camera menu (Apple side)

Danny, on the iPhone 17 Pro with 0.9.0: the app crashes when switching to the front camera, "huge
performance lag", later "I can't select any other options", and the front picture turned 90° to
the right in portrait and landscape. **No change to `shared/`.**

- **Crash.** Crash log: EXC_BAD_ACCESS in `padImage` (AltruisticBodyPoseKit) under
  `VNHumanBodyPose3DDetector` — Vision's own code, on the front camera's square 1280×1280 format,
  once a body is in view. The app remembers the camera, so it then crashed at every start. Vision
  now gets a fixed upright 720×1280 / 1280×720 picture with the camera's intrinsics (`VisionCanvas`;
  docs/03 "The picture Vision gets"), new requests when the shape changes, and a steady roll.
- **Picture a quarter turn off.** The mounting check compared against the system's level angle
  alone; the capture connection had already turned the frames by 270° (docs/03 "Front camera").
  Now the difference is used (`L2KCameraView.remainingTurn`).
- **Camera menu.** Every picture republished the whole app state, so the page was redrawn 15 times
  a second with the menu in it. In the simulator (camera choices listed with `-l2kFakeSources 1`)
  the menu opened but a tap on a choice did nothing. The per-picture state is now its own object
  (`LiveState`) observed only by the preview, the readings and the diagnostics card, and the menu
  is an equatable view.
- **Lag.** Not reproduced: with nobody in view the ultra-wide (1920×1440 and 1280×720) and the
  front camera all ran at 30 fps with 21–23 ms of work per frame, 15 preview pictures a second and
  the main queue idle; with a body in front of the front camera the same. The page being redrawn
  15 times a second (above) is the one thing found that could be felt as lag. New diagnostics:
  "Timing: picture … ms old on arrival, … ms of work per frame, device cool/warm/hot" and, with
  `-l2kLog 1`, "Screen: … pictures per second, the main queue up to … ms behind".
- New switches: `-l2kFormat 1280x720` (force a capture format), `-l2kVisionCamera 0` (no intrinsics
  for Vision), `-l2kFakeSources 1` (simulator only).
- Verified: `swift test` 18 tests (3 new: the picture's layout and intrinsics, the drawn picture
  against the layout for the four rolls, the camera detector through turns and formats on a
  picture with `L2K_TEST_IMAGE`); `make -C shared test` 1136 checks. **On the iPhone 17 Pro**
  (`-l2kLog 1` over `devicectl`): formats chosen 1920×1440 (rear) and 1280×1280 (front), 30 fps;
  front camera with a body in view: no crash, 15 joints, "Camera mounting: as assumed; roll agrees
  with the system in 99 of 103 checks (level angle 0°, frames already turned 270°)".
- **Not verified:** the menu on a device (no way to tap from the Mac: Xcode's device interaction
  offers simulators only; Danny to confirm); the front picture's direction by eye; that Vision on
  iOS uses the intrinsics (385 px against 494 px given in the one close-up reading); distances
  against a tape measure; the turn between portrait and landscape with a body in view; the iPad.
  Log sessions started with `devicectl … launch --console` end the app when the console is closed.

## 2026-11-11 (3) — the camera's whole view, front camera (Apple side)

Danny: "Make ultrawide detect in full view, add the front camera option." **No change to `shared/`**
(a front camera frame is `L2K_SOURCE_VISION_3D`, like the ultra-wide's).

- **Whole view.** `UltraWideSource` ran the 1280×720 preset, a 16:9 cut of the 4:3 sensor (measured
  105° × 73°). It now picks the capture format with the largest view (`L2KCameraView.widestFormat`:
  largest view in tangent space at 30 fps, frames up to 2048 px wide, the smallest one at least
  1280 px wide), keeps geometric distortion correction on (the maths assumes a pinhole picture),
  and switches off everything that crops: stabilisation, zoom, Center Stage (front). For a 4:3
  format at 105° that is 105° × 89°. Vision already got the whole frame.
- **Preview.** The app's colour preview for these cameras showed twice the Kinect's view (about
  96° × 80°, cutting the picture's sides). It now zooms out exactly as far as the whole camera image
  needs, for every roll (`L2KCameraView.zoomToFit`, `L2KSession.localColorPreviewFitsCamera`).
- **Front camera.** `UltraWideSource(lens: .front)`: the camera on the screen's side with the widest
  view, same Vision 2D + 3D pose and bone-length depths as the ultra-wide, picture not mirrored.
  Where "up" is in a front camera's picture depends on how the sensor is mounted, which differs
  between devices; the assumed mounting is checked against the system's own level angle
  (`AVCaptureDevice.RotationCoordinator`) and corrected when they steadily disagree
  (`L2KCameraMounting`; the rear camera only reports the comparison). App 0.9.0: "Front camera" in
  the camera list (now a menu), `-l2kSource front`; `-l2kLog 1` lists both lenses' formats.
- Verified: `swift test` 15 tests (3 new: format choice, preview fit for the four rolls, mounting
  from the level angle); the app builds for iOS. **Not run on a device** (iPad and iPhone were
  locked or out of reach): the format actually chosen and its measured view, the front camera's
  mounting check, the frame rate with the larger frames, and body detection through the front
  camera are all unverified. First thing to read on a device: the "Format …" and "Camera
  mounting …" lines of the live diagnostics.

## 2026-11-11 (2) — normalising for Dance Evolution (Apple side)

- Danny asked for an option that sends the skeleton at the optimal position for the game.
- **`shared/` addition** (`docs/03`, "Normalising"): `l2k_map_params.normalize`, `norm_players`,
  `norm_distance` 2.5, `norm_spacing` 0.61, `norm_follow_seconds` 2, `norm_reach` 0.5; the mapper
  levels the frame, puts one player at (0, 2.5 m) or two at ±0.61 m, keeps short movements, drops
  other bodies. `l2k_mapper.picture` (pre-fit skeletons) replaces `fit_shift`. The fit is now a final
  pass over tracked and held bodies. No packet change. 1136 checks.
- App 0.8.0: "Best position for Dance Evolution" with 1 / 2 players (off by default; `-l2kNormalize 1`,
  `-l2kNormalizePlayers 2`); the picture overlay draws the real skeletons, the top view the reported
  ones. Checked in the simulator with two test figures. **Not tried in the game or on a person.**
- `L2KReceiver` now hands out one frame without bodies when the sender goes away.

## 2026-11-11 — `L2KReceiver`: a game port can take its sensor from another device (Apple side)

- Danny asked for the Dance Evolution iPad port to accept the sensor feed of another device.
- `ios/Sources/L2KKit/L2KReceiver.swift`: the receiving end of L2K1 in Swift. Opens the UDP port
  (24810 by default), optionally advertises `_l2k._udp`, answers HELLO with CONFIG
  (`L2K_CONSUMER_GAME_PORT`), takes one sender at a time (a second device is ignored while the first
  is alive, 3 s timeout, BYE honoured), drops late or repeated skeleton datagrams, reassembles and
  decodes the depth image (D1 or raw) and the JPEG colour image, and hands out `L2KFrame`s like an
  in-process `L2KSession` does. No protocol change.
- Verified: `testReceiverGetsFramesFromSender` (the real sender against the receiver on localhost:
  two tracked skeletons, depth with player pixels, 640x480 colour, frames in order, back to listening
  after BYE). Not yet run between two devices.

## 2026-11-10 (late night, 6) — preview overlay follows the real player again (Apple side)

- 0.7.0 drew the skeleton over the camera picture from the skeleton the game is told about. With the
  play-area fit moving a player who stands outside the box, the 2D points sat off the person.
- `shared/`: `l2k_mapper.fit_shift[slot]` = what the fit added to each body in the last frame (state
  only; no packet or convention change). The app's overlay subtracts it; the top-down inset keeps
  showing the reported position. 1037 checks. Not checked with a person in view.
- App 0.7.1.

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
