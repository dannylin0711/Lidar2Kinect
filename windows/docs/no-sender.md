# What the game sees without a sender

`l2k.ini` → `no_sender = missing | idle`. Default: **missing**.

## What the game does (read from gamekdm.dll)

The Nui thread (`0x1000ebd0`) registers a device status callback (`0x1000ebb0`), sets its own
"retry" event once and then loops: wait for the retry event → `NuiGetSensorCount`, create,
initialise, open streams → run while `NuiStatus()` succeeds → on any failure shut the sensor down
and go back to waiting for the retry event. **The callback sets the retry event only for
`hrStatus == S_OK`.** So the game is built to sit in "sensor missing" until the runtime reports a
sensor through the callback, and to recover by itself when one is plugged in again.

## The two modes

| | missing (default) | idle |
|---|---|---|
| no sender | `NuiGetSensorCount` → 0; `NuiStatus` → `E_NUI_NOTCONNECTED`; game state 4 "sensor missing" | a sensor is always there; empty skeleton frames, black images at 30 Hz |
| sender appears | status callback `S_OK` → the game reopens the sensor | frames start to contain the player |
| sender leaves (BYE or 3 s silence) | callback `E_NUI_NOTCONNECTED`, `NuiStatus` fails → the game shuts the sensor down and waits | frames become empty again |

## Observed in the game (2026-11-10, spice2x, fake sender, Danny watching)

- **missing**, sender already running at boot: the game opened the sensor 0.5 s after its first
  call (count 0 → HELLO → callback → count 1) and ran; no sensor error.
- **missing**, sender stopped while the game ran: the game showed a **sensor error**; when the
  sender was started again the game reopened the sensor within a second and **the error cleared
  by itself**, camera image and figure back.
- **idle**, game booted with no sender: **no error, empty camera view**; when the sender started,
  the player appeared.
- Not observed: **missing** with no sender during the whole boot (the game's own logic handles it
  — state 4 and wait for the callback — but what the attract mode / boot checks show then was not
  watched).

## Choice

`missing` stays the default: it is how a real Kinect behaves when unplugged, the game handles it
with its own code path, the error tells the player the phone is not connected, and it clears
itself when the app connects. `idle` is the option for setups that would rather never show a
sensor error (the game then looks like a Kinect that sees nobody).
