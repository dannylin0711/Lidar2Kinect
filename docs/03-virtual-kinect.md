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

Quality bits: a joint beyond 97 % of the half field of view sets the matching clipped-edge bit.
Which side the SDK calls "left" and "right" is taken to be the image's left and right — not yet
checked against a real sensor.

**Floor plane** `(A, B, C, D)`: `(A, B, C)` is the up vector in skeleton space and `D` the
sensor's height above the floor. The height is estimated from the lowest tracked ankle or foot
joint; it follows a lower reading quickly and a higher one slowly, so a jump does not raise the
floor. `l2k_mapper_set_floor` replaces the estimate with a measured height.

**Elevation angle** (`l2k_elevation_degrees`): `asin(up.z)`, positive when the sensor looks
upwards.

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
| Ultra-wide (`UltraWideSource`) | the rear ultra-wide camera on a plain capture session (no ARKit, no LiDAR); **Vision 3D body pose** on the image; gravity from the motion sensors | 1 | none measured: Vision's person mask at the body's distance | written 2026-11-10; ran on an iPhone 17 Pro at 30 fps with a body seen; accuracy not assessed |
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
standing person fits from about a metre away. Vision's 3D request gives each joint's position in
the image and its distance along the optical axis; the 3D point is rebuilt with the camera's own
intrinsics (`VisionBody3DDetector.place`), so it does not depend on Vision's axis conventions.
Vision assumes a body height of 1.80 m when it cannot measure one, so distances are right in
proportion but only roughly in scale. The Kinect images still have the Kinect's field of view:
they show the middle of what the camera sees, while the skeleton carries every joint (those outside
the image set the clipped-edge bits). The app's colour preview zooms out to twice the Kinect's view
and frames the Kinect's own image with a dashed rectangle.

**Ultra-wide, depths.** Device snapshots showed the 3D request's pose sitting off the body, so its
depths are not used either. Each joint is placed on its image ray at the bone's length from its
parent joint (`VisionKinematics`); of the two points that satisfy that, the 3D model only decides
which — towards the camera or away. Bone lengths are the 3D model's, averaged over time. The root
(pelvis) keeps the model's distance. Tested on the Mac against a known body; not yet judged on a
person.

**Measured on the iPhone 17 Pro (2026-11-10).**

- ARKit: world tracking + LiDAR depth yes, + person mask yes, + 2D body no; 3D body tracking yes,
  but with neither depth nor mask. `sceneDepth` is nil while camera tracking is still "limited"
  (the first seconds, or a phone lying still) and appears once tracking is normal: 256×192, 100 % valid.
- LiDAR source: 60 ARKit frames/s, 30 processed, Vision 2D pose 10–20 ms.
- Ultra-wide source: 30 frames/s, Vision 3D pose 17 ms, 23 ms with the person mask.
- Vision's left / right are the person's own: on a frame of five dancers facing the viewer, every
  "left" shoulder and hip is on the image's right (`testVisionOnAPicture`).
