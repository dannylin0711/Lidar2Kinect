# l2k.ini and l2k.log

Both live next to the DLL (`Kinect10.dll` or `l2k_spicehook.dll`). Every key is optional.

```ini
[l2k]
port=24810          ; UDP port the receiver listens on
log=l2k.log         ; relative to the DLL's folder; "none" = no log
no_sender=missing   ; missing | idle   - what the game sees with no sender (no-sender.md)
depth=auto          ; auto | sender | skeleton
                    ;   auto: the sender's depth image while it sends one (< 0.5 s old),
                    ;         else drawn from the skeleton (l2k_depth_rasterize)
depth_mode=players  ; players | full  - asked of the sender: depth on player pixels only, or all
color_quality=0     ; JPEG quality asked of the sender, 1..100, 0 = the sender's default
dnssd=1             ; advertise _l2k._udp so the app lists this PC (Windows 10 1809+; else ignored)
verbose=0           ; 1 = log every frame call, not only the 5-second summaries
name=               ; shown in the app; default: the computer name
```

DanceAround's `visionposewrapper.dll` reads the same `[l2k]` keys (port, log, dnssd, verbose, name)
plus a `[vp4u]` section: `../vp4u/README.md`.

Environment variables override the file: `L2K_INI` (another ini path), `L2K_PORT`, `L2K_LOG`.
The tests use them to stay off the default port.

## The log

Rewritten at each start. Lines are `[seconds thread] text`:

- `game: …` — every call the game makes that changes state (`NuiInitialize`, stream opens,
  tracking enable/disable, tracked-skeleton changes, angle set, shutdown), and every 5 s a summary:
  frames taken / "no data" per stream, `NuiStatus` calls, angle gets, smoothing calls, ordinal 9
  calls, frame-end events.
- `receiver: …` — senders appearing (address, name, capabilities), BYE or 3 s of silence, the
  streams asked for, and every 5 s the rates received, packets lost, late or malformed, and JPEG
  decode failures.
- `status: …` — sender presence changes and the device status callback sent to the game.

A typical good session: `NuiGetSensorCount -> 0`, then `HELLO from …`, `status: sender present ->
device status callback`, `NuiGetSensorCount -> 1`, `NuiInitialize(0xb)`, two `NuiImageStreamOpen`,
`NuiSkeletonTrackingEnable(…, 0x2)`, then summaries of about 150 frames per stream per 5 s.
