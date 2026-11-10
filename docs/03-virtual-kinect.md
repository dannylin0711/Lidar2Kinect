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

Joint state: source confidence 2 → tracked, 1 → inferred, 0 → not tracked (the joint then sits
on its parent joint). A body with a pelvis (or both hips) and a neck is *tracked*; with less it
is *position only*.

Slots: a body keeps its slot (0…5) while its tracking id stays; a body that disappears frees its
slot at once.

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

| Source | Camera | Bodies | Status |
|---|---|---|---|
| ARKit body tracking (`ARKitBodySource`) | rear, LiDAR improves scale | 1 | written, builds; **not yet run on a device** |
| Synthetic figure (`SyntheticSource`, `l2k_synth_body`) | none | 1–2 | working; used by tests and the simulator (level camera 1.0 m up) and by `l2k_fake_sender` (`l2k_synth_body_at`: 2.0 m up, tilted −24°, like the cabinet's Kinect; `-H` / `-a` change it) |
| Vision body pose + LiDAR depth | rear or front | several | planned: needed for two players and for the front camera |

Whether ARKit delivers scene depth and person segmentation together with body tracking depends
on the device and OS; the source asks ARKit at run time and the app shows what it got. Without
them the depth image is drawn from the skeleton (a capsule figure per player).
