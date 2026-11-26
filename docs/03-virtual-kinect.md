# The virtual Kinect

What the translation layer pretends to be: a Kinect v1 sensor standing where the iPhone or iPad
camera is. This document fixes the conventions; `shared/src/l2k_map.c` and `l2k_depth.c`
implement them and `shared/tests/test_l2k.c` checks them.

## 1. Principle

The layer behaves like a physical Kinect, not like an idealised one. If the phone is tilted, the
skeleton coordinates are tilted with it, the reported elevation angle says by how much, and the
floor plane says where the floor is — exactly what a tilted Kinect reports. The game then applies
whatever correction it applies to a real sensor (it reads the elevation angle and the floor
plane, see `docs/01`).

Only the roll is normalised: the phone may be mounted in either landscape or in portrait, and the
virtual Kinect is always upright.

## 2. Spaces

**Source camera space** (what capture sources report; this is ARKit's camera space): origin at
the camera, +x to the right of the sensor's native landscape image, +y up in that image, +z
towards the viewer — the camera looks along −z. Metres.

**Kinect skeleton space** (what the layer outputs): origin at the sensor, +z out of the sensor
towards the player, +y up, +x to the sensor's left. A player facing the sensor has their right
hand at +x. Right-handed, metres.

**Roll quadrant** `q`, from the direction of gravity in the source image
(`l2k_quadrant_from_up`): 0 = the image's up is up (native landscape), 1 = the image's right is
up, 2 = the image's down is up, 3 = the image's left is up. The app only changes `q` when the
device is clearly past the diagonal.

Source → Kinect (`l2k_camera_to_kinect`), with `z_k = −z` always:

| q | x_k | y_k |
|---|---|---|
| 0 | −x | +y |
| 1 | +y | +x |
| 2 | +x | −y |
| 3 | −y | −x |

## 3. Images

The virtual Kinect has one pinhole camera for both images:

- focal length **285.63 px at 320×240** — the constant the Kinect SDK headers use to project
  skeleton points into the depth image, so the game's own projection of a joint lands on the
  right pixel. Field of view 58.5° × 45.6°.
- images are **mirrored**, as Kinect images are: a point at +x (the player's right, when facing
  the sensor) is on the right of the image.
- depth is 320×240, colour is 640×480 from the same camera: depth pixel (x, y) and colour pixel
  (2x, 2y) show the same point.

Skeleton point → depth pixel (`l2k_skeleton_to_depth_pixel`):
`u = 160 + 285.63 · x / z`, `v = 120 − 285.63 · y / z`.

The phone's wide camera is expected to see more than this in landscape (roughly 65° × 51°; the
real figure comes from the camera intrinsics and has not been read on a device yet), so the
virtual image is a crop of the camera image. In portrait the camera's horizontal view is then
narrower than the Kinect's 58.5° and the left and right edges of the virtual image are empty.
**Mount the device in landscape.**

`l2k_virtual_to_source` maps a virtual pixel to the source image using the camera's intrinsics;
the depth image, the person mask and the colour image are all resampled through it, so they stay
registered.

Depth pixels: `depth_mm << 3 | player_index`. Depth outside 800…4000 mm is 0. Player index n
means skeleton slot n−1.

## 4. Skeleton

Capture sources report anatomical joints (`l2k_src_body`, 19 joints, hands and toes optional).
The mapper produces the 20 Kinect joints:

| Kinect joint | From |
|---|---|
| shoulder centre, head | neck base, head centre |
| shoulders, elbows, wrists, hips, knees, ankles | the same source joints |
| hip centre | pelvis + `hip_center_lift` × (neck − pelvis) |
| spine | pelvis + `spine_fraction` × (neck − pelvis) |
| hand | the source's palm joint, else wrist + `hand_extension` × (wrist − elbow), marked inferred |
| foot | the source's toe joint, else a point `foot_length` in front of the ankle, marked inferred |

Joint state: source confidence 2 → tracked, 1 → inferred. A joint the source does not give (0)
is reported **inferred**, at its last known offset from its parent joint, or on the parent if it
was never seen. A tracked skeleton therefore never carries a "not tracked" joint. The reason is
the game: Dance Evolution drops a player to the rest pose unless all 20 joint states are nonzero
(read from the DLL in the port, `docs/16` there). A body with a pelvis (or both hips) and a neck
is *tracked*; with less it is *position only*.

**Upper-body mode** (`l2k_map_params.upper_body`, the app's "Upper body only" switch, off by
default) is for a camera that sees the player from about the waist up:

- a body is tracked from its neck and both shoulders; a pelvis is not required — when the source
  has none it is placed straight below the neck, 1.4 × the shoulder width away;
- the legs are never taken from the source: the hips sit 0.26 × the shoulder width either side of
  the pelvis, the ankles 1.6 × the neck-to-pelvis length straight below them, the knees half way,
  all reported inferred;
- every frame carries the flag `L2K_FRAME_UPPER_BODY`, so a receiver can tell the legs are made
  up; there is no floor estimate.

The arms, head and shoulders are the player's own. Whatever the game judges on the legs (foot
targets, step notes, the leg part of a pose) is judged against that standing pose: the layer does
not fake leg movement, so those notes miss unless the dancer happens to stand the same way. The
three ratios are estimates.


**Dropouts and identity.** A camera-based source loses a body far more easily than a Kinect (one
missed detection is enough), and the game follows its player by tracking id — Danny observed that
a player who is lost and comes back is not accepted as the same player, and the game does not go
on. So the mapper owns the identities, not the source:

- the tracking ids it reports are its own (1, 2, …);
- a body missing for up to `hold_seconds` (0.5 s) is still reported in its slot, with its last
  skeleton and every joint inferred: a short gap is invisible to the game;
- after that the slot is empty, but with `keep_identity` (on by default; the app's "Keep the same
  player" switch) the slot and tracking id stay reserved, and the next body the source reports
  takes over the reserved identity nearest to where it stands. With it off the identity is dropped
  when the hold ends and a returning body gets a new id, as on a Kinect.

This is a deliberate departure from a real Kinect, which gives a re-entering player a new id.
What exactly the game does when its tracked id disappears has not been read from the DLL yet.

**Play area.** The game only takes a body as a player inside a box in front of the sensor —
1.80 to 3.20 m away, 1.25 m either side, hip 0 to 2 m above the floor it is told about, with 5 cm
of slack to be taken on and 20 cm to be kept (`docs/01` §7). A wide lens invites standing closer
than that. The game's judging does not use where the player stands, so with `fit_play_area` (the
app's "Fit me into the game's play area", on by default in the app, off in the library) the mapper
slides a tracked body that is outside the box horizontally to 10 cm inside its edge, joints and
`Position` together, and keeps the floor it reports consistent with the body (hip between 0.4 and
1.6 m above it) when there is no measured floor. A body already inside is not touched. The depth
and colour images are not changed, so in this case they no longer line up with the skeleton.

**Normalising for Dance Evolution.** The game places its world 2.5 m in front of the sensor: the
middle of the play-area box, and where its dancer stands; its two-player rule expects the players
0.61 m either side of that (`docs/01` §7). With `normalize` (the app's "Best position for Dance
Evolution", off by default) the mapper reports, whatever the room looks like:

- a level sensor: every joint is turned about the sensor so that up is the y axis, `up` is sent as
  (0, 1, 0) (elevation 0) and the floor plane as (0, 1, 0, sensor height). Heights above the floor
  are unchanged;
- each player on a standing point: one player at x = 0, z = 2.5 m; two players at x = −0.61 m and
  +0.61 m, each on the side they really stand on. The standing point follows the player's own
  position with a time constant of 2 s (`norm_follow_seconds`; 0 pins the hips), so a step or a
  lean is sent as it is and only the place in the room is removed. A player is never reported more
  than 0.5 m (`norm_reach`) from the standing point, hence never outside the play area;
- only `norm_players` (1 or 2) players, those whose identity is the oldest. Other bodies are not
  sent as skeletons (they are still in the depth image's player index).

It replaces the fit when on. `l2k_mapper.picture` keeps every skeleton as it was before the fit or
the normalising, for drawing over the camera picture. Not yet tried in the game: whether any judge
depends on where the player stands rather than on the pose is not read from the DLL.

**The game decides the play area (since 2026-11-26).** The box above and the 2.5 m standing point
are Dance Evolution's. DanceAround (`windows/vp4u`) takes a player whose spine base is 1.80–2.81 m
from the phone and within 0.9 m of its axis (its play area with windows/vp4u's default placement of
the phone, `windows/docs/vp4u-interface.md` §5), and it takes one player, the nearest. The receiver
says which game it feeds by its CONFIG `consumer_kind` (5 = DanceAround, anything else = Dance
Evolution; `l2k_game_of_consumer`), and `l2k_map_params_set_game` sets the play area and the standing
point to match:

| | fit keeps the hips within | standing point | reach | players |
|---|---|---|---|---|
| Dance Evolution | 1.90–3.10 m, ±1.15 m | 2.5 m (two players ±0.61 m) | 0.5 m | 1 or 2 |
| DanceAround | 1.90–2.70 m, ±0.80 m | 2.3 m | 0.4 m | 1 |

The app follows the connected receiver (`L2KSession` sets `L2KPipeline.game`) and keeps the last
receiver's game while none is connected; `-l2kGame 1` starts it on DanceAround. If `phone_x` /
`phone_z` in windows/vp4u's `l2k.ini` move the phone, DanceAround's box moves with them and the app
does not know: the fit then keeps to the wrong box.

Quality bits: a joint beyond 97 % of the half field of view sets the matching clipped-edge bit.
Which side the SDK calls "left" and "right" is taken to be the image's left and right — not yet
checked against a real sensor.

**Floor plane** `(A, B, C, D)`: `(A, B, C)` is the up vector in skeleton space and `D` the
sensor's height above the floor. The height is estimated from the lowest tracked ankle or foot
joint; it follows a lower reading quickly and a higher one slowly, so a jump does not raise the
floor. `l2k_mapper_set_floor` replaces the estimate with a measured height.

**Elevation angle** (`l2k_elevation_degrees`): `asin(up.z)`, positive when the sensor looks
upwards.

## 4b. The 30-joint body

DanceAround's skeleton is Kinect v2's 25 joints and five face points (`docs/02` §4b,
`l2k_body_frame`). The mapper makes it from the same frame as the Kinect v1 skeleton
(`l2k_mapper_body_frame`): same slots, ids, states, positions, fit, normalising and hold.

- Joints 0–19 are the skeleton's joints; the skeleton's shoulder centre is the body's spine shoulder.
- Ten more: the neck, both hand tips and thumbs, the nose, eyes and ears. A capture source may give
  the last nine (`L2K_SRC_NOSE` … `L2K_SRC_THUMB_RIGHT`); they are then sent with the source's
  confidence (2 tracked, 1 inferred). The Kinect v1 skeleton is the same with or without them.
- Whatever the source does not give is placed from the skeleton and reported **inferred**:

| Joint | From |
|---|---|
| neck | shoulder centre + 0.35 × (head − shoulder centre) |
| hand tip | hand + 1.2 × (hand − wrist) |
| thumb | wrist + 0.8 × (hand − wrist) + 3 cm forward |
| nose | head + 8.5 cm forward − 2 cm up |
| eyes | head + 7.5 cm forward + 3 cm up ± 3.2 cm sideways |
| ears | head ± 7.5 cm sideways − 1 cm forward |

  "Forward" is where the shoulders face (perpendicular to the level shoulder line), "sideways" the
  player's right. These are the Windows side's first estimates (they were `windows/vp4u/vp4u_map.cpp`'s),
  moved here so both ends use the same; not compared with VisionPose or a Kinect v2.
- A receiver given only SKELETON gets the same derived joints with `l2k_body_frame_from_skeleton`.
- The hold, the fit and the normalising move the ten joints with the skeleton; a held body's are
  reported inferred.

What the capture sources give:

| Source | Face | Hand tips, thumbs |
|---|---|---|
| LiDAR depth | Vision 2D nose, eyes, ears, lifted with the depth map (nose and ears on the surface, eyes 1 cm behind it) | placed |
| Ultra-wide, front camera | Vision 2D nose, eyes, ears, each on its own ray at a typical distance from the head centre as placed (nose and eyes 8.7 cm, on the camera's side; ears 7.6 cm, beside the head) | placed |
| Body tracking (ARKit) | ARKit's nose and eye joints, inferred unless ARKit tracks them | placed (ARKit's hand joints follow the forearm) |
| Synthetic figure | its own: the head turns ±20° from side to side, which only the face shows | its own |

Not measured: whether DanceAround uses the face or the hands for anything but drawing.

## 5. Smoothing

`l2k_smoother_apply` is the Holt double-exponential joint filter from Microsoft's skeletal joint
smoothing white paper, with the SDK's parameter names. It stands in for `NuiTransformSmooth`,
which the game calls on every frame with the SDK default parameters (0.5, 0.5, 0.5, 0.05, 0.04).
It has not been compared with the real DLL's output.

## 6. Calibration (not done yet)

These numbers are first estimates, not measurements:

| Parameter | Default | Meaning |
|---|---|---|
| `hip_center_lift` | 0.10 | how far above the pelvis the Kinect puts the hip centre, as a fraction of the torso |
| `spine_fraction` | 0.30 | where the Kinect puts the spine joint along the torso |
| `hand_extension` | 0.30 | hand position past the wrist, as a fraction of the forearm |
| `foot_length`, `foot_drop` | 0.13 m, 0.06 m | foot joint relative to the ankle |
| head centre offset (ARKit source) | 0.09 m | from ARKit's head joint to the centre of the head |

To calibrate: record the same person in the same poses with a real Kinect v1 and with the app,
and fit the parameters. Until then the joints that depend on them are plausible but not matched.

## 7. Capture sources

What ARKit supports was read on an iPad Pro (M2) and an iPhone 17 Pro on 2026-11-10 (the app's
"This device" card): world tracking gives the LiDAR depth map, **but not together with 2D body
detection**; 3D body tracking is supported, **but with neither the depth map nor a person mask**.
ARKit alone therefore cannot give a body and LiDAR depth in the same session.

| Source | Where the body comes from | Bodies | Depth image | Status |
|---|---|---|---|---|
| LiDAR depth (`VisionDepthSource`) | ARKit world tracking for the image, intrinsics, gravity and the LiDAR depth map; **Vision** 2D body pose on the camera image, lifted with the depth map | up to 2 | the LiDAR depth map; players from ARKit's person mask where it combines with scene depth, else Vision's | written 2026-11-10; coordinate handling tested on the Mac; **not yet run on a device** |
| Ultra-wide (`UltraWideSource`) | the rear ultra-wide camera on a plain capture session (no ARKit, no LiDAR); **Vision 3D body pose** on the image; gravity from the motion sensors | 1 | none measured: Vision's person mask at the body's distance | written 2026-11-10; ran on an iPhone 17 Pro at 30 fps with a body seen (1920×1440 since 0.9.0); accuracy not assessed |
| Front camera (`UltraWideSource(lens: .front)`) | the widest camera on the screen's side, otherwise as the ultra-wide; picture not mirrored; the sensor's mounting checked against the system's level angle | 1 | as the ultra-wide | written 2026-11-11; ran on an iPhone 17 Pro (0.9.1) at 30 fps, 1280×1280, with a body seen and the mounting check agreeing; not played with |
| Body tracking (`ARKitBodySource`) | ARKit's model-based 3D skeleton | 1 | no depth map on these devices: drawn from the skeleton | ran on Danny's devices without a skeleton appearing; cause not established |
| Synthetic figure (`SyntheticSource`, `l2k_synth_body`) | generated | 1–2 | drawn from the skeleton | working; used by tests, the simulator and `l2k_fake_sender` |

**LiDAR depth, step by step.** Vision's body-pose request runs on each camera frame (at most 30
per second; a frame already 150 ms old is skipped), told how the device is rolled so it sees
people upright. Its points come back in the upright image and are turned into buffer coordinates
(`VisionOrientation`, tested for the four rolls with a QR code). Each joint then gets the depth
the LiDAR measured at its pixel (`DepthLift`): the median over the person pixels of a 5×5 window,
plus a per-joint surface-to-joint offset; a limb joint more than 1 m from the torso's depth is
held at the torso's depth and reported as estimated. People keep their id from frame to frame by
nearest position (`VisionPersonTracker`); the two largest are sent. With two players, each mask
pixel goes to the player whose body centre is nearest by image column.

Consequences: the skeleton is metric and the depth image is real; a joint hidden behind the body
gets the depth of what hides it; the head joint is placed from the ears or the nose. Vision's
left and right are taken to be the person's own — the app draws the left side orange and the
right side blue so this can be checked at a glance on a device.

**Ultra-wide.** The LiDAR covers only the normal wide camera's view, and ARKit offers only that
camera (every world- and body-tracking video format on the iPhone 17 Pro is the wide-angle camera).
The ultra-wide camera measured 105° × 73° at 1280×720, against the Kinect's 58.5° × 45.6°, so a
standing person fits from about a metre away. Since 0.9.0 the source uses the camera's whole view:
the format with the largest view (a 4:3 format reads the whole sensor, about 105° × 89°; chosen by
`L2KCameraView.widestFormat`, frames up to 2048 px wide), distortion correction on, no
stabilisation, zoom or Center Stage. The view the device really delivers is in the live
diagnostics ("Format …", "Field of view …"); on the iPhone 17 Pro it is 1920×1440, and the frame's own
intrinsics give 95–103° × 79–86° (they move with the focus). Vision's 3D request gives each joint's position in
the image and its distance along the optical axis; the 3D point is rebuilt with the camera's own
intrinsics (`VisionBody3DDetector.place`), so it does not depend on Vision's axis conventions.
Vision assumes a body height of 1.80 m when it cannot measure one, so distances are right in
proportion but only roughly in scale (a person 1.62 m tall is reported 11 % too far). The Kinect images still have the Kinect's field of view:
they show part of what the camera sees, while the skeleton carries every joint (those outside
the image set the clipped-edge bits). Which part: see "Where the images look" below. The app's colour preview zooms out until the whole camera
image shows (`L2KCameraView.zoomToFit`) and frames the Kinect's own image with a dashed rectangle.

**Where the images look (0.9.2, every source).** When the skeleton is sent as the camera sees it,
the depth and colour images are the middle of the camera picture at the Kinect's field of view.
When the skeleton is moved — the play-area fit, or the normalising — the images move with it
(`L2KGameView`, `L2KPipeline.imagesFollowSkeleton`, on by default): the joints of the sent skeleton
and of the skeleton as the camera sees it are both projected into a 320×240 Kinect image, and the
images are scaled (by the ratio of how far the two sets of points spread) and shifted (by their
centres) inside the camera picture until the player lies where the sent skeleton is. The scale and
shift are applied as another focal length and principal point of the source camera
(`L2KGameView.camera`; the virtual-to-source mapping is affine), eased over a few frames, and
return to the plain view when nobody is tracked. Before 0.9.2 a player normalised to 2.5 m while
standing close to a wide camera was sent with a picture of their face (Danny, 2026-11-11). Depth
values are not changed: a measured depth map still carries the real distances. The app's preview
frames the part the game gets with the dashed rectangle. `-l2kImagesFollow 0` switches it off.

**The picture Vision gets (0.9.1).** Not the camera buffer: the buffer turned upright for the
device's roll and fitted, with black bars, into a fixed 720×1280 picture (1280×720 when the upright
image is wider than high), drawn by `VisionCanvas`; Vision's points are mapped back through
`VisionCanvasLayout`. Three reasons, all seen on 2026-11-11:
- Vision's 3D request crashed inside its own padding (`padImage`, AltruisticBodyPoseKit) on the
  front camera's square 1280×1280 frames as soon as a body was in view (iPhone 17 Pro, iOS 27.3).
- A 3D request that has seen a wide picture fails on a tall one and the other way round ("Invalid
  X-dimension 288/192" on macOS 27, where a new request works again; on the iPhone the app twice
  printed "E5RT … Surface Object's width does not match" and exited while being picked up). New
  requests are made when the picture's shape changes, and the roll used for Vision only changes
  when the picture is clearly turned (a device lying flat keeps the roll it had).
- Without intrinsics Vision assumes a focal length of 0.76 × the picture's height (measured on
  macOS 27 at five picture sizes; the same body's distance doubles when the focal length given
  doubles). With an ultra-wide camera every distance was therefore about twice too large. The
  picture's own intrinsics are now given (`VNImageOption.cameraIntrinsics`); on the Mac Vision
  then works with the focal length given to within 2 %. On the iPhone the one reading with a body
  (close, upper body only) was 385 px used against 494 px given — **not settled on a device**; the
  diagnostics line "Picture for Vision …" shows both. `-l2kVisionCamera 0` leaves the intrinsics out.

**Front camera.** The same source on the camera of the screen's side, for a player in front of the
device's own screen. Its picture is taken unmirrored, so it is a camera facing the player like any
other and Vision's left / right stay the person's own. World up comes from the motion sensors in
device axes; a front camera looks along +z(device), and its picture is assumed to have x along
-y(device) and y along -x(device). Because that mounting differs between devices, the roll worked
out from gravity is compared with the angle the system gives for a level picture of that camera;
a steady disagreement turns the assumed mounting by the missing quarter turns (`L2KCameraMounting`).
The system's angle is the turn the capture connection has to apply to the camera's own frames; the
frames delivered have already been turned by the connection's angle, so what is compared is the
difference (`L2KCameraView.remainingTurn`). On the iPhone 17 Pro's front camera the level angle is
0° in portrait and the connection's angle is 270°: the frames arrive lying on their side, as the
assumed mounting says (0.9.0 took the 0° alone and showed the picture a quarter turn off).

**Ultra-wide, depths.** Device snapshots showed the 3D request's pose sitting off the body, so its
depths are not used either. Each joint is placed on its image ray at the bone's length from its
parent joint (`VisionKinematics`); of the two points that satisfy that, the 3D model only decides
which — towards the camera or away. Bone lengths are the 3D model's, averaged over time. The root
(pelvis) keeps the model's distance. Tested on the Mac against a known body; not yet judged on a
person.

**Limb depths since 0.9.4 (camera-only sources).** Measured with the Dance Evolution port's sensor bench
(its `docs/47`): the port's dancer rendered from a front camera (known pose in every picture) and two
videos of players, run through this detector. With the 3D model choosing towards / away, the limbs'
tilt out of the picture plane came out with the wrong sign more often than not (rendered dancer:
forearms −25° true, +27° measured; thighs −15° / +15°; shins +16° / −24°; median errors 59° forearm,
42° shin), and the 2D detector's joints do not match the 3D model's bone lengths (its thigh ~15 %
shorter, its shin up to ~15 % longer), so a limb lying in the picture plane was lifted out of it. The
game compares the 3D directions of the arms and legs on 3/4 of its pose notes, so those notes failed.
0.9.4 (`VisionHybridDetector.DepthRule.adaptive`, `LengthSource.picture`):

- bone lengths: the 90th percentile over time of the bone's span in the picture (both joints seen, at
  the parent's model distance), not the model's length;
- a limb bone (upper arm, forearm, thigh, shin) lies parallel to the picture unless the picture shows
  it shorter than 0.8 (arms) / 0.75 (legs) of its length; a shortened upper arm, forearm or thigh is
  placed towards the camera, a shortened shin away from it (a player facing the screen reaches
  towards it; a knee bends backwards). The torso, shoulders and hips keep the model's choice.

Rendered dancer, median angle error per limb (two songs, two camera placements): upper arm 31–41° →
20–22°, forearm 35–60° → 24–35°, thigh 13–37° → 10–34°, shin 30–43° → 9–23°. In the game (bench): pose
notes 75 % → 100 % and shake ends 19–25 % → 78 % on two videos of players; position notes unchanged. `-l2kDepthModel 1`
(app) / `UltraWideSource.depthFromModel` returns to the 0.9.3 rule. Not yet run on a device.

**Measured on the iPhone 17 Pro (2026-11-10).**

- ARKit: world tracking + LiDAR depth yes, + person mask yes, + 2D body no; 3D body tracking yes,
  but with neither depth nor mask. `sceneDepth` is nil while camera tracking is still "limited"
  (the first seconds, or a phone lying still) and appears once tracking is normal: 256×192, 100 % valid.
- LiDAR source: 60 ARKit frames/s, 30 processed, Vision 2D pose 10–20 ms.
- Ultra-wide source: 30 frames/s, Vision 3D pose 17 ms, 23 ms with the person mask.
- Vision's left / right are the person's own: on a frame of five dancers facing the viewer, every
  "left" shoulder and hip is on the image's right (`testVisionOnAPicture`).

**Delay (camera-only sources, since 0.9.3).** What the game gets is late by the camera's own delay
(exposure to the frame arriving in the app) plus Vision's pose. Two things keep the rest out of the
skeleton's way:

- *The skeleton goes first.* `L2KPipeline.skeletonFrame` makes the Kinect skeleton; `L2KSession`
  sends its packet and calls `onSkeleton` before any image exists. The person mask is made on
  demand (`L2KSourceFrame.makeSegmentation`, asked for by the depth image), then the depth and
  colour images, then their packets and `onFrame`. The packets and their order are unchanged; the
  skeleton only leaves earlier. A consumer that judges movement takes the skeleton from
  `onSkeleton` and the images from `onFrame`.
- *A 60 fps camera, 30 skeletons a second* (`UltraWideSource(captureRate: 60)`). The camera runs
  at 60 frames a second in its widest format that fast; the body is still detected 30 times a
  second, on every second frame (`L2KCameraView.usesFrame`), so the game's smoothing sees the
  Kinect's rate. A receiver that asks for more (CONFIG `rate`, DanceAround: 60) gets it: the session
  asks the source for that rate (`L2KSource.requestFrameRate`) and the body is detected on every
  frame Vision keeps up with; the images stay at 30 a second. With the camera at 30 the receiver gets
  30, and the app says so. A frame of a 60 fps stream reaches the app sooner after its exposure. The fast
  format can show less (`UltraWideSource.fastCameraKeepsView`); the app turns the faster camera on
  by itself only where it shows the same ("Faster camera", `-l2kFastCamera 0|1`).

Measured on an iPad Pro 11" (M2), front camera 1920×1440 (107° × 91° at both rates), nobody
dancing, 2026-11-22, from the app's "Timing" line:

| | picture old on arrival | Vision pose | skeleton handed on after the picture was taken |
|---|---|---|---|
| 30 fps, 0.9.2 order (mask first on every second frame) | 46–48 ms | 18–21 ms | 64–68 ms, +6–7 ms on every second frame |
| 30 fps, skeleton first | 46–48 ms | 18–21 ms | 64–68 ms |
| 60 fps, skeleton first | 28–32 ms | 17–23 ms | 44–52 ms |

iPhone 17 Pro front camera: 60 ms on arrival at 30 fps in 1280×1280 (measured 2026-11-11); its
60 fps format is 1024×768 with 84° × 68° instead of 84° × 84° — **not measured at 60 fps yet**.
Not measured anywhere: the whole chain to the game's screen.
