# vp4u — Lidar2Kinect for DanceAround

A replacement for DanceAround's `VisionPoseWrapper.dll` (x64, vp4u API 210). The game gets its
skeleton from the Lidar2Kinect app over the network instead of from two RealSense cameras and
VisionPose, so it needs no RealSense, no CUDA / TensorRT and no VisionPose licence — and runs on any
GPU. The interface and the reasons behind every choice: `../docs/vp4u-interface.md`.

The wrapper tells the app it is DanceAround's receiver (`consumer_kind` 5: the app uses
DanceAround's play area) and asks for 60 frames a second of SKELETON + BODY, the 30-joint body in
the game's own joint order (`../../docs/02-wire-protocol.md` §4b). Each frame's BODY is levelled,
placed in the game's coordinates and passed on; a sender without BODY (an app before 0.10.0) or a
lost BODY datagram falls back to the SKELETON frame with the ten extra joints placed from it
(`l2k_body_frame_from_skeleton` in `shared/`).

## Build and test

Visual Studio 2022 (its bundled CMake), **64-bit**:

```
cmake -S windows/vp4u -B build/vp4u -A x64
cmake --build build/vp4u --config Release
ctest --test-dir build/vp4u -C Release --output-on-failure
```

Output in `build/vp4u/bin/Release/`: `visionposewrapper.dll`, `vp4u_probe.exe`, `test_vp4u.exe`,
`l2k_fake_sender.exe`, `l2k_dump.exe`. Tests: `l2k` (shared golden packets), `vp4u_map` (the
mapping: axes, levelling, floor, play area, BODY against SKELETON), `vp4u_probe` (the DLL driven
like the game, fed by `l2k_fake_sender` over loopback: 60 frames a second from BODY) and
`vp4u_probe_skeleton` (the same with `body=0`: from SKELETON).

## Install

In the game folder (the one with `spice64.exe`):

1. Rename `game\dancearound_data\plugins\x86_64\visionposewrapper.dll` to
   `visionposewrapper.dll.orig`.
2. Copy `visionposewrapper.dll` (and optionally an `l2k.ini`, below) into that folder.
3. Open UDP port 24810 in Windows Firewall for the game (below).

`ee-config.xml` can stay as it is: the loader still loads the CUDA / VisionPose DLLs it lists, but
nothing calls them (seen in the game).

To go back: delete the new DLL and rename `.orig` back.

### Firewall: UDP 24810

The app sends its first packet (HELLO) to the PC, so the game must be allowed to receive on UDP
24810. Windows Firewall blocks inbound traffic for a program without an allow rule, and then the
phone looks as if it sends nothing: the app gets no answer and `l2k.log` shows no `receiver:` line
at all. One rule, for the game's `spice64.exe` only, on private networks — in a PowerShell **run as
administrator**, with the path of your game folder:

```powershell
New-NetFirewallRule -DisplayName "Lidar2Kinect (DanceAround spice2x)" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 24810 -Program "F:\BEMANI\UDN-001-2022062801\spice64.exe" -Profile Private
```

- The PC's network must be a private network (Settings → Network & internet → the connection →
  Private), or the rule does not apply; `Get-NetConnectionProfile` shows it.
- A different `port` in `l2k.ini` needs the same number here.
- The game need not be restarted after adding the rule.
- Check: `Get-NetFirewallRule -DisplayName "Lidar2Kinect (DanceAround spice2x)"`. To remove it:
  `Remove-NetFirewallRule -DisplayName "Lidar2Kinect (DanceAround spice2x)"`.

On the iPhone, the app also needs **Settings → Privacy & Security → Local Network → Lidar2Kinect**
on; without it iOS stops the app from sending to the PC. The app finds the PC by Bonjour
(`_l2k._udp`, the computer's name) or by its address on the network.

When it works, the game log has `l2k: sender "…" connected (offers BODY; …)` and `l2k.log` a
`receiver: last 5 s: …` line every 5 s.

## Where to put the phone

The game's coordinates come from the cabinet's left camera (`../docs/vp4u-interface.md` §5). The
wrapper puts the phone at the middle of the play area, on the cameras' line. So:

- phone **centred** in front of where you dance, rear camera towards you, landscape;
- you stand about **2.3 m** from it — the game only sees a player whose hips are within ±0.5 m of
  that distance (and ±0.9 m sideways); to stand elsewhere set `phone_z` = 2.3 − your distance;
- any height and tilt: the wrapper levels the skeleton and takes the phone's height from the floor
  the app estimates (or from `phone_height`).

## l2k.ini

Next to the DLL; every key optional. `[l2k]` is the receiver's (`../docs/config.md`; its
`no_sender`, `depth*`, `color_quality` keys are for the Kinect and unused here).

```ini
[l2k]
port=24810
log=l2k.log
dnssd=1

[vp4u]
no_sender=idle        ; idle: cameras always reported active, empty frames until the app connects
                      ; missing: cameras reported inactive without the app (the game raises error 11/12)
camera_height=auto    ; metres, height of the game's origin; auto = LeftCameraPose.Y of the game's
                      ;   VisionPoseForUnityConfig.json (else 1.16)
phone_x=0.55          ; metres, the phone's position along the game's x axis (0.55 = play area centre)
phone_z=0             ; metres, along the depth axis; 2.3 - (your distance to the phone)
phone_height=auto     ; metres above the floor; auto = from the app's floor estimate
default_phone_height=1.0 ; used when auto and the app has no floor estimate yet
yaw=0                 ; degrees about the vertical axis, if sideways steps come out diagonal
mirror_x=0            ; 1 swaps the player's left and right (docs/vp4u-interface.md open point 1)
video_width=480       ; the placeholder camera image (test mode, play video)
video_height=480
calibration_file=dev/nvram/stereo_cam.yml ; written so the game sees the cameras as calibrated
                      ;   (it raises CameraErrorUncalibrated 5-1503-1001 otherwise); "none" = never
pin_d3d10warp=1       ; keep Windows' D3D10Warp.dll loaded: works around a crash in the game's video
                      ;   playback (docs/vp4u-interface.md §6b); 0 = leave it alone
rate=60               ; skeleton / body frames a second asked of the app (0 = its default, 30);
                      ;   the cabinet's cameras ran at 60
body=1                ; ask for the 30-joint BODY stream; 0 = SKELETON only, the ten extra joints
                      ;   placed on this PC
```

The calibration file is a placeholder: the game only checks that it exists. It lives in the game's
virtual file system (an encrypted nvram mount), so the wrapper writes it through `kamunity.dll`,
the game's own bridge to that file system. If you put the original `visionposewrapper.dll` back,
run the camera calibration in test mode again.

## The log

`l2k.log` next to the DLL, and the same lines (prefixed `l2k:`) in the game's own log through
VisionPose's logger (`game\dev\raw\log\output_log.txt`, and spice's `log.txt`). Under spice2x
the loader routes the DLL's file calls through the game's VFS, where an absolute path may not
exist; the wrapper then falls back to `dev/raw/log/l2k.log` inside the VFS. The game log's
`l2k: module …; ini …; log …` line says where the log went and whether `l2k.ini` was found. Every call that changes state (`vp4uGetApiTable`, `Initialize`,
`InitVisionPose*`, `OpenRealSense`, analysis / preview / calibration start and stop, recording),
the sender connecting or going away, and every 5 s: pose frames per second, how many were empty,
bodies per frame.

## Try it without an iPhone

```
build\vp4u\bin\Release\vp4u_probe.exe -t 10 --sender build\vp4u\bin\Release\l2k_fake_sender.exe
```

or, with the game running, `l2k_fake_sender.exe -n 1 -t 3600 <the game PC's address>`: the fake
figure stands 2.5 m from a phone 2 m up, tilted −24°, which lands in the play area.
