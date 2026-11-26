# DanceAround's vp4u interface (VisionPoseWrapper.dll, API 210)

What DanceAround (Konami, UDN, build 2022062801, Unity 2020.3.18f1, x64) needs from
`VisionPoseWrapper.dll`, the native plugin that turns two RealSense cameras + NEXT-SYSTEM's
VisionPose into skeletons. `windows/vp4u` replaces it with one fed by Lidar2Kinect.

Sources, in this order of weight:

- **M**: the game's managed code, `dancearound_data\Managed\VisionPoseForUnity.dll`
  (`Vpw4U`, `Vp4uApiNativeInterop`, `VisionPoseManager`, `Vp4uCoordinateMapper`, `BodyPose`,
  `VisionPoseHelpers`, `VisionPoseCameraPreview`) and `Konami.GameSystem.dll`
  (`MotionCaptureSystem`, `SystemAvatar`, `RealSenseActiveChecker`), read from their metadata and
  IL. Type::method names below say where each fact is.
- **N**: the original `VisionPoseWrapper.dll` (exports, a few instructions of each export).
- **C**: the game's configuration: `prop\udnappsetting.json`, `vpprops\visionposeforunityconfig.json`,
  `prop\ee-config.xml`.
- **L**: a log of the original wrapper running under spice2x on an RTX 3070 without cameras
  (`game\dev\raw\log\output_log.txt` of a third-party setup).

No game code is reproduced here; only the facts.

## 1. Loading

- The boot loader (`ee`) loads the DLLs listed in `ee-config.xml <vp4u><load_libraries>` in
  order (CUDA, TensorRT, RealSense, OpenCV, `visionpose.dll`, `visionpose2.dll`,
  `visionposewrapper.dll`, libaio…), pins `visionposewrapper.dll` and calls `vp4uPreboot` with
  `<vp4u_config_path>`, retrying up to `<boot><retry_count>` = 12 times on failure (C, from the
  file's comments). Unity reads the module's handle from its command line, `-vp4uh <hex>`
  (`VisionPoseManager::LoadPluginModule`, M); presumably the loader puts it there.
- Exports, by name: `vp4uGetApiTable` @1, `vp4uGetVersion` @2, `vp4uPreboot` @3,
  `vp4uShutdown` @4. (N)
- `vp4uGetVersion()` returns 210. `vp4uGetApiTable(version, table*, log_cb, log_ctx)` returns 0
  unless version is 210 and table is non-NULL, then fills the table. `vp4uPreboot(const char *config, …)`
  loads that config and returns 1 on success (N). Under spice2x the string is the **text** of
  `VisionPoseForUnityConfig.json`, not its path (seen in the game, run 3). What its other two
  arguments are is not known (the replacement ignores them).
- The C# side (`Vpw4U::LoadPlugin`): `vp4uGetApiTable(210, &table, log_cb, NULL)` — a 0 throws
  "Failed to GetApiTable." — then `RegisterLoggerCallback(log_cb, NULL)`, then `Initialize()`; a
  NULL instance means "not loaded". (M)
- Calling convention: every delegate is `[UnmanagedFunctionPointer(Cdecl)]` (one convention on
  x64); string parameters are `LPStr` (ANSI `char*`); `bool` returns have no `MarshalAs`, i.e. a
  4-byte BOOL. (M)

## 2. The function table (29 pointers, this order)

| # | Function | Requirement seen in the game (M) |
|---|---|---|
| 0 | `uint64 GetCurrentTimestamp()` | microseconds; the C# `Timestamp` divides by 10⁶; frame timestamps use the same clock |
| 1 | `void* RegisterLoggerCallback(cb, ctx)` | result ignored |
| 2 | `void UnregisterLoggerCallback(handle)` | |
| 3 | `void AddRefImageDesc(ImageDesc*)` | called when the C# side clones a frame's image |
| 4 | `void ReleaseImageDesc(ImageDesc*)` | called when it releases it |
| 5 | `void* Initialize()` | non-NULL |
| 6 | `void Shutdown()` | |
| 7 | `void LoadConfig(char* path)` | path of `VisionPoseForUnityConfig.json` (L: `vpProps/VisionPoseForUnityConfig.json`, relative to the working directory) |
| 8 | `void ReloadConfig()` | |
| 9 | `BOOL OpenRealSense()` | false throws "Failed to open camera" → init state 106 (failed) |
| 10, 11 | `BOOL RealSenseLeftIsActive()`, `…RightIsActive()` | see §4 |
| 12 | `BOOL TryRebootRealSense(BOOL)` | false throws |
| 13 | `BOOL TryRebuildRealSenseFilters()` | |
| 14, 15 | `int InitVisionPoseRealtime(char* productKey)`, `…Offline(…)` | must return exactly 1, else "Failed to init vision pose" (state 105) |
| 16 | `BOOL StartRealtimeCameraPreview(leftCb, leftCtx, rightCb, rightCtx)` | test-mode camera check |
| 17 | `void StopRealtimeCameraPreview()` | |
| 18 | `BOOL StartStereoCameraCalibration(statusCb, previewCb, ctx)` | test mode |
| 19 | `BOOL CommitStereoCameraCalibration()` | false throws |
| 20 | `void CloseStereoCameraCalibration()` | |
| 21 | `BOOL StartRealtimeAnalysis(poseCb, ctx)` | false throws |
| 22 | `void StopRealtimeAnalysis()` | the C# delegate is dropped right after it returns |
| 23 | `BOOL PrepareVideoRecording(char* path)` | called from the game's play scene (`GamePlaySceneController`); false throws |
| 24–26 | `BOOL StartVideoRecording()`, `FinalizeVideoRecording()`, `DeleteRecordedVideo(char*)` | |
| 27 | `BOOL StartOfflineAnalysis(char* path, poseCb, ctx)` | not called by the game's own code |
| 28 | `void StopOfflineAnalysis()` | |

Callbacks (first argument: the `ctx` given with them; the game passes NULL):

- log: `(ctx, char* message)`
- video: `(ctx, uint64 timestamp_us, ImageDesc*)`
- calibration status: `(ctx, int status, int current, int total, char* message)`; the C# side
  computes progress `0.95 · current / total`, or 1 for status 3. Status 1 = in progress,
  3 = success (`VisionPoseManager.CalibrationStatus`).
- pose: `(ctx, uint64 timestamp_us, ParsedBody* pose3d, int n, ParsedBody* pose2d, int n,
  ParsedBody* pose2dMain, int n, ParsedBody* pose2dSub, int n, ImageDesc* mainVideo, ImageDesc* subVideo)`.
  **`mainVideo == NULL` means "end of stream"**: the C# side then passes null to its handler instead
  of a frame. A realtime frame must therefore always carry a non-NULL main image.

## 3. Data

```
ImageDesc  { void* pixels; int width, height, bitsPerPixel, bytesPerPixel, bytesPerRow, bytesPerFrame; }  32 bytes
Joint      { int state; float x, y, z; }                                                              16 bytes
ParsedBody { int state; int pad; int64 trackingId; Joint joints[30]; }                               496 bytes
```

- Joint order: Kinect v2's `JointType` (0 SpineBase … 24 ThumbRight), then 25 Nose, 26 EyeLeft,
  27 EyeRight, 28 EarLeft, 29 EarRight. States 0 not tracked, 1 inferred, 2 tracked. (M)
- `BodyPose::ImportFromNative`: a body is tracked when `state > 0`; a joint position is valid when
  its `state > 0` (inferred is as good as tracked); no rotations come from the wrapper. (M)
- Only `pose3d` reaches the game. `pose2dMain` / `pose2dSub` are read only by the 2D debug
  visualiser; `pose2d` by nothing found. (M)
- The camera image: `VisionPoseCameraPreview::UpdateImage` makes an **RGB24** texture of
  width × height and loads `bytesPerFrame` bytes into it (M). The original showed the cameras'
  480 × 480 crops (C: `LeftCameraRoi`, `RightCameraRoi`).

## 4. Boot sequence and camera status (M)

`VisionPoseManager.InitializeAsync`: LoadPlugin → `LoadConfig` → `InitVisionPoseRealtime` →
`InitVisionPoseOffline` → if `RealSenseLeftIsActive() && RealSenseRightIsActive()` is false,
`OpenRealSense()` → `StartRealtimeAnalysis` → ready (state 7). Any failure leaves VisionPose not
ready.

`Konami.GameSystem.IO.Device.RealSenseActiveChecker::Check`: when VisionPose is ready and the
debug option *ConnectDummyCamera* is off, left inactive → error code 11, right inactive → 12.
Whether that error clears by itself when the cameras come back is **open**; the replacement
reports both cameras active by default (`no_sender=idle`).

**Calibration file.** `Konami.GameSystem.IO.StereoCameraCalibrationHelper.CalibrationIsRequired`
is true unless *ConnectDummyCamera* is on or the file **`dev/nvram/stereo_cam.yml`** exists in the
game's VFS (`GamePath.kVp4uStereoCameraParameterVfsPath`; M). When it is required the game goes to
error mode, **`CameraErrorUncalibrated 5-1503-1001`** (seen in the game, 2026-11-26, with the
first build: "stereo camera calib: file is not found: dev/nvram/stereo_cam.yml", "Camera is not
calibrated"). Test mode's camera check commits a calibration (`CommitStereoCameraCalibration`) and
then backs the file up to `mnt/tmp/stereo_cam.yml` (`StereoCameraCalibrationHelper.BackupCalibrationData`).
The game only checks that the file exists.

`/dev/nvram` is not a plain folder: `avs-config.xml` mounts it as an `nvram2` file system
(mirrored, encrypted, hashed file names under `game\dev\e|f\CONF\UDN\dev\nvram\_NVRAM_`), and
`GameFs.IsFileExistVfs` asks avs: `KAMUNITY.Avs.FsFileIsExist` = `FsLstat(path)` succeeds and the
stat's mode is 2 (a file), with the path passed unchanged (`dev/nvram/stereo_cam.yml`) (M). A plain
file does not count: run 2 showed the wrapper's own file calls are **not** redirected into the VFS
(no import hooked; `dev/...` paths landed under the working directory, `…\game\`), and the game
still did not find the file.

So the replacement writes the placeholder through `kamunity.dll` — the game's native bridge to avs,
loaded in the process — with its exports `FsOpen(char* path, int flags, int mode)` → handle > 0,
`FsWrite(int fd, void*, int len)`, `FsClose(int fd)`, `FsLstat(char* path, AVS_FS_STAT*)` → nonzero
if it exists (signatures from KAMUNITY's delegates; the native `FsOpen` passes its arguments
straight to avs and records the handle that `FsWrite` / `FsClose` check). Flags 50 and mode 0644
are what `KAMUNITY.Avs.FsOpenW` uses. `AVS_FS_STAT`: three u64 times, u32 link count, size,
high size, u16 mode (offset 36), u16 permission. Written at `LoadConfig` if missing, rewritten at
`Commit`; without `kamunity.dll` (the probe) a plain file instead.

## 5. Coordinates

What the game does with `pose3d` (M):

- `Vp4uCoordinateMapper::TransformBody3D` with the game's settings (`udnappsetting.json`:
  `isMirror` true, scale 1, rotations 0, `VisionPoseManager` at the origin) leaves the positions
  unchanged: it multiplies x by −1 only when `isMirror` is false.
- `VisionPoseHelpers::FindForemostPose` picks the body with the smallest SpineBase **z** that is
  greater than the manager's z: **z is the distance from the camera, growing towards the player.**
- `SystemAvatar` takes that body's **SpineBase**, and `MotionCaptureSystem` checks it against the
  play area of `udnappsetting.json`: centre (0.55, 0, 2.30), size (1.6, 2.0, 0.81), margin 0.1.
  So a player is seen when SpineBase is within x 0.55 ± 0.9, y 0 ± 1.1, **z 2.30 ± 0.505**.
- Unity is y-up and nothing flips y, so **y is up**.

What the cabinet looked like (C): two cameras 1.2 m apart (`LeftCameraPose` x 0.30, `RightCameraPose`
x 1.50), 1.10 m (cabinet) / 1.16 m (third-party setup) above the floor, both facing the stage
(yaw 180); `CameraIndex Main` = the left camera; `MainCameraDistance` 2.5 m. The left camera's crop is
the left part of its image (x 32–512 of 848), the right camera's the right part (320–800).

Taken together — **inferred, not observed**:

- origin at the main (left) camera, at its height: the play area's y = 0 ± 1.1 then holds a
  SpineBase about 0.2 m below camera height; z 2.30 is a player standing ~2.3 m from the cameras;
- **+x points from the left camera to the right one**, and x 0.55 is between them;
- the left camera's crop on the left of its image means the player stands to that camera's left,
  i.e. the "left" camera is on the player's left: **+x is the player's right** (facing the
  cameras) — the same as Kinect skeleton space. With `isMirror` true the game keeps it as is.

The replacement (`vp4u_map.cpp`) therefore levels the Kinect skeleton (removes the phone's pitch
and roll using the frame's `up`), keeps Kinect's axes, and moves the phone to
(`phone_x` 0.55, phone height − camera height, `phone_z` 0): a player 2.3 m in front of a phone
centred on the play area lands in the middle of the game's play area.

## 6. Derived joints

Kinect v1 has 20 joints; the vp4u body has 30. The replacement places the others (marked
inferred, so the game uses them), from estimates — **not compared with VisionPose output**:

| Joint | From |
|---|---|
| SpineShoulder | Kinect ShoulderCenter |
| Neck | ShoulderCenter + 0.35 × (Head − ShoulderCenter) |
| HandTip | Hand + 1.2 × (Hand − Wrist) |
| Thumb | Wrist + 0.8 × (Hand − Wrist) + 3 cm forward |
| Nose, eyes, ears | Head ± fixed offsets along the body's forward / sideways / up axes (forward from the shoulder line) |

Phase 2 (a 30-joint stream from the phone, ARKit's own hand and face joints) replaces these.

## 7. Open

1. **Left / right.** §5's "+x is the player's right" rests on the camera crops; not yet seen in
   the game. Check: raise the right arm; the avatar should mirror it like a mirror would. If it
   is the wrong side, `mirror_x=1`.
2. **Origin.** Camera height and the 2.3 m distance are inferred from the play-area box. Check:
   standing 2.3 m from the phone, the game sees the player; stepping 0.5 m forward or back loses
   them.
3. **Rate.** The original ran 60 fps (`CaptureFrameRate`); Kinect-shaped frames are 30 fps. The
   game filters joints itself (`Vp4uKalmanPose`); how judgement copes with 30 fps is not known.
4. **Play video.** Nothing is recorded; what the game does when the file it named in
   `PrepareVideoRecording` does not exist after `FinalizeVideoRecording` is not known.
5. **Error 11 / 12** recovery (§4) with `no_sender=missing`.
6. `vp4uPreboot`'s second and third arguments.
7. Not run with the real iPhone app (only `l2k_fake_sender`).
8. ~~Calibration file through kamunity~~ — confirmed in run 3 (§8).

## 8. Seen in the game

2026-11-26, first build, spice2x, RTX 4070 Ti, no sender:

- spice's vp4u loader loaded and pinned the DLL, read version 210, called `vp4uPreboot`.
- Through the game's own log (`l2k:` lines): `vp4uGetApiTable(210)`, `Initialize`, `LoadConfig`
  (main camera height 1.16 m read from the game's config), `InitVisionPoseRealtime` / `Offline`,
  `StartRealtimeAnalysis` — VisionPose ready; `OpenRealSense` not called (cameras reported active).
- Test mode's camera check and calibration screens open; the calibration screen shows the
  wrapper's message; preview, calibration, commit, close called as expected.
- Then `CameraErrorUncalibrated` (§4) — fixed in the second build, to be confirmed.
- `l2k.log` did not appear next to the DLL. The second build falls back to `dev/raw/log/l2k.log`.

Run 2 (second build): `0 of 102 imports redirected by the loader`; the log fell back to
`dev/raw/log/l2k.log`, which is a plain file `…\game\dev\raw\log\l2k.log` (working directory
`…\game`); opening `l2k.log` next to the DLL fails for a reason not known (the plugins folder).
The placeholder went to the plain `…\game\dev\nvram\stereo_cam.yml`, so `CameraErrorUncalibrated`
again → third build writes through kamunity (§4).

Run 3 (third build, no sender): "calibration file dev/nvram/stereo_cam.yml: written to the game's
VFS"; no `CameraErrorUncalibrated`; the game went GameBoot → TestMode → GameModeEntryPoint →
NetworkConfirm → GameGaishuDemo (attract). Only `NetworkErrorServicesError 5-2002-0910` (no
e-amusement server). The wrapper delivered empty pose frames at ~24.5/s for the whole run.
`vp4uPreboot`'s first argument turned out to be the config's text (§1).

Also seen in run 2: a `NullReferenceException` in the
game's own `Testmode4Unity.TMScene.Start` / `SuperFactorySetting.Start` (test mode), not in a
wrapper call; its cause is not known.
