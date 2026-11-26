# Brief for the Windows agent

You are picking up the Windows half of **Lidar2Kinect**. This file is your starting prompt; the
person who opened this session (Danny) owns the project and will answer questions, but everything
that can be read from the repository or from the game's files you should read yourself.

## DanceAround: what the Apple side added (2026-11-26) — read this first if you work on `vp4u`

The asks 2–4 of `ios/CLAUDE.md` are done in `shared/` and the app (0.10.0), in the layout you proposed;
`docs/04-status.md` "2026-11-26 (2)" has the details and what was verified. All additions: version 1,
existing golden packets unchanged, `body.bin` new. Nothing in `windows/` was touched. Until vp4u adopts them
it keeps getting exactly what it gets today.

What is new in `shared/`:

- `L2K_CONSUMER_VP4U` = 5. `l2k_game_of_consumer(kind)` → `L2K_GAME_DANCEAROUND` for 5, Dance Evolution for
  every other kind; the app then fits the player into DanceAround's area (1.80–2.80 m, ±0.9 m from the phone,
  assuming your default `phone_x` 0.55 / `phone_z` 0) and "Best position" puts one player at 2.3 m.
- `l2k_config.rate` (byte 43, was `reserved`): skeleton / body frames a second wanted, 0 = 30. The app sends
  up to that rate (60 needs its "Faster camera" on for the camera-only sources); images stay at 30.
- BODY: `L2K_PKT_BODY` 0x13, `L2K_STREAM_BODY` 0x8, `l2k_body_frame` / `l2k_body` with
  `L2K_BODY_JOINT_COUNT` 30 in vp4u's joint order (`L2K_BODY_*` = your `VP4U_*`), `l2k_encode_body` /
  `l2k_decode_body` (`docs/02` §4b). Same header, space, slots, ids and states as SKELETON; same
  `frame_number` as the SKELETON frame it goes with, sent right after it.
- `l2k_body_frame_from_skeleton(&skeleton_frame, &body_frame)`: the 30 joints from a SKELETON frame, with
  the derived joints of your `vp4u_map.cpp` (same formulas and constants, now in `l2k_map.c`, using the
  frame's `up` instead of a levelled y; identical after levelling). For an older app that offers no BODY.
- `L2K_SRC_JOINT_COUNT` is now 28 (face and hand points after the 19 skeleton joints;
  `L2K_SRC_SKELETON_COUNT` 19). `l2k_synth_body` fills them all (the figure's head turns ±20°, only its face
  shows it), so `tools/smoothcmp.cpp`'s noise loop now also shakes those points — harmless, the Kinect
  skeleton is built from the first 19 only.
- `l2k_fake_sender` offers BODY, sends it when asked, and follows CONFIG `rate`; `l2k_dump -s 9 -r 60 -k 5`
  asks the way vp4u should.

Suggested steps for `windows/vp4u` (your call):

1. `init_module(module, L2K_CONSUMER_VP4U, …)` in `vp4u_wrapper.cpp`.
2. CONFIG `rate` 60 for vp4u, 0 for `kinect10` / `spicehook` (e.g. a `rate` key in `[vp4u]`, default 60,
   passed through the core's `send_config`).
3. Ask for `L2K_STREAM_SKELETON | L2K_STREAM_BODY` (the sender drops what it does not offer); the core decodes
   BODY (a `hub_body` next to `hub_skeleton`); vp4u uses the BODY frame when it has one for the newest frame
   number, else `l2k_body_frame_from_skeleton` — one mapping path from `l2k_body_frame`: level, place, mirror
   the 30 joints. Your derived-joint code can go; the `vp4u_map` test should still hold through
   `l2k_body_frame_from_skeleton`.
4. The dispatcher already turns every new frame into one pose frame, so 60 a second needs no change there;
   the "pose frames/s" log line will show it.
5. Build `shared/` with MSVC (x86 and x64) and run `test_l2k` (1470 checks on macOS): **not yet built with
   MSVC**. Then `windows/STATUS.md` and a line in `docs/04-status.md`.

## What this project is

Dance Evolution Arcade (Konami, build `KDM-2016021900`) is a dance game played with an Xbox 360
Kinect through Microsoft's Kinect for Windows SDK v1 (`Kinect10.dll`). Danny runs the game on a
Windows PC with spice2x and has no wish to depend on a real Kinect. An iPhone or iPad with LiDAR
tracks the player instead, and the **Lidar2Kinect app** streams the result — already converted
to Kinect conventions — over the local network.

Your job is the receiving end on Windows: make the game believe a Kinect is attached and feed it
what the app sends. Two packagings of the same core are wanted:

- **2b — fake Kinect:** a replacement `Kinect10.dll` the game loads in place of Microsoft's.
- **2a — spice2x hook:** a DLL loaded by spice2x alongside the game that redirects the game's
  Kinect calls to the same core.

A macOS agent has already built the shared library, the protocol and the iOS app. Nothing on the
Windows side exists yet, and **the shared code has never been compiled on Windows**.

## Read these first

| File | Why |
|---|---|
| `README.md`, `docs/00-overview.md` | the parts and how they fit |
| `docs/01-dea-kinect-surface.md` | **the specification of your work**: every Kinect call the game makes, with arguments, addresses in `gamekdm.dll`, the event and status behaviour it depends on, and what is still unverified |
| `docs/02-wire-protocol.md` | the packets you receive |
| `docs/03-virtual-kinect.md` | units, axes, image conventions of the data |
| `shared/include/l2k/*.h` | the C API you link: decoding, reassembly, depth codec, smoothing, a synthetic figure |
| `shared/tools/l2k_dump.c` | a complete receiver in 250 lines — the model for yours |
| `CLAUDE.md` | rules for changing anything in `shared/` |

## Facts you can rely on, and facts you must check

Established from the game's DLL (details and addresses in `docs/01`):

- `gamekdm.dll` is a **32-bit** DLL. Everything you build that loads into the game is x86.
- It imports exactly five functions from `Kinect10.dll`, **by ordinal**: 5, 6, 9, 20, 22.
- It creates sensor 0, initialises it with flags `0x0B`, opens depth+player-index at 320×240 and
  colour at 640×480, enables skeleton tracking, and runs a thread that waits on events.
- The stream and skeleton events are manual-reset and the game does not reset them; it expects a
  frame-end event at least every 30 s; it maps specific HRESULTs of `NuiStatus` to its sensor
  states.

Not established — settle these before building on them:

1. **Ordinal → function names.** `docs/01` §2 infers them from the call shapes. Confirm with
   `dumpbin /exports` on a real 32-bit `Kinect10.dll` (Kinect for Windows Runtime v1.8 is a free
   Microsoft download; the 32-bit DLL lands in `SysWOW64`). Record the full export table: a
   replacement must put every export at its real ordinal.
2. **Struct layouts and vtable order.** `docs/01` gives sizes and offsets seen in the game
   (`NUI_SKELETON_FRAME` 2664 bytes, `NUI_IMAGE_FRAME` 40 bytes, `INuiSensor` method offsets,
   `INuiFrameTexture::LockRect` at `+0x14`). Check them against the SDK's own headers with
   `static_assert`s in your code.
3. **How the game finds `Kinect10.dll`** in Danny's spice2x setup (which directory wins in the
   DLL search order), and how spice2x loads hook DLLs and when, relative to `gamekdm.dll`. Read
   spice2x's documentation and source; do not assume.
4. **Whether spice2x already does anything about this game's Kinect.** Read its source for the
   game's module before designing the hook.
5. **`NuiTransformSmooth`.** The game calls it on every skeleton frame. `l2k_smoother_apply` is
   the published algorithm, not a copy of the DLL's code. With the real `Kinect10.dll` available
   you can call its `NuiTransformSmooth` on synthetic frames (no sensor needed for that call, if
   it turns out to work without one) and compare. Report the difference; fix `shared/` if it is
   systematic.

Ask Danny at the start, in one message: where the game and spice2x are installed, which Windows
version and Visual Studio are on the machine, whether the Kinect runtime is installed, and
whether a real Kinect is available for comparison.

## What to build

Work in `windows/`. Suggested layout — change it if you find a better one, and say why:

```
windows/
  CMakeLists.txt        builds everything as Win32; pulls in ../shared
  core/                 static library: receiver + virtual sensor
  kinect10/             2b: Kinect10.dll (core + .def with the real ordinals)
  spicehook/            2a: hook DLL for spice2x
  probe/                test program that uses the Kinect API the way the game does
  docs/                 your findings (export table, load order, measurements)
  STATUS.md             your running status
```

### Step 0 — make the shared code work on Windows

Build `shared/` with CMake for Win32 and run its tests: the golden packets in
`shared/tests/vectors` must match byte for byte. Then run `l2k_dump` and `l2k_fake_sender`
against each other on this machine. This is the first MSVC build of that code; fix what breaks
(following `CLAUDE.md`) before anything else. You now have a sender that needs no iPhone.

### Step 1 — the core

A static library with two halves:

- **Receiver.** A thread with one UDP socket (default port 24810, configurable). It answers every
  HELLO with a CONFIG asking for the streams the game has opened, keeps the newest skeleton
  frame, reassembles depth and colour images (`l2k_assembler`), decodes them (depth codec D1 is
  in `shared/`; colour is JPEG — use WIC or a single-file decoder, no large dependency), and
  notices when the sender goes silent (3 s).
- **Virtual sensor.** The `INuiSensor` object and the five flat functions, behaving as `docs/01`
  §3–§6 describe: correct vtable order with `E_NOTIMPL` for what the game never calls; image
  frames through an `INuiFrameTexture` whose `LockRect` gives the pixel buffer; events set when a
  new frame is ready and reset when it is taken; a frame-end event each frame; `NuiStatus` and
  the device status callback reflecting whether a sender is present; the elevation angle taken
  from the skeleton frame's `up` vector (`l2k_elevation_degrees`), with "set angle" accepted and
  ignored; ordinal 9 answering `colour = 2 × depth`; ordinal 20 running `l2k_smoother_apply`.

Decide and document: what the game should see while no sender is connected. Try the candidates
against the real game (sensor missing until the first HELLO, then the status callback; or sensor
present with empty frames) and keep the one that gives the best behaviour in the game's own UI.

When the sender provides no depth image, draw one from the skeleton with `l2k_depth_rasterize`,
so the game's player silhouette still works.

### Step 2 — the probe

A small x86 program that drives the Kinect API exactly as the game does — same imports by
ordinal, same call order, same events and waits (`docs/01` §3–§4) — and prints what it receives.
It must run against your replacement DLL, and against Microsoft's DLL if a real Kinect is
present. It is how you prove compatibility without the game, and it is the regression test.

### Step 3 — 2b, the fake Kinect

`Kinect10.dll`, x86, exporting the real ordinals (all of them; the ones the game does not use
return `E_NOTIMPL`). Verify with the probe, then with the game under spice2x fed by
`l2k_fake_sender`: the game leaves its sensor-error state, the skeleton moves in test mode or in
a song, the player silhouette and camera image appear.

Do this before 2a: the game imports `Kinect10.dll` statically, so on a machine without
Microsoft's runtime the game cannot load at all until a `Kinect10.dll` exists.

### Step 4 — 2a, the spice2x hook

The same core in a DLL loaded through spice2x's hook mechanism, redirecting the five imports of
`gamekdm.dll`. Work out, from spice2x and from testing, how the import of a possibly missing
`Kinect10.dll` is satisfied in this mode, and state plainly in your status what the hook adds
over 2b on Danny's setup. If it adds nothing, say so rather than building it for its own sake.

### Along the way

- A plain-text config file next to the DLL (port, log file, depth mode) and a log that says what
  the game called and what the sender sent, enough to diagnose a session after the fact.
- Optionally advertise `_l2k._udp` by DNS-SD so the app lists the PC. Load that API dynamically:
  the DLL must still work where it is missing.

## Constraints

- x86 builds, static C runtime, no installer-level dependencies.
- Declare the Kinect interfaces and structs you need yourself, from the documented ABI. Compiling
  against an installed SDK's headers is fine; **do not copy Microsoft's headers or binaries, the
  game's files, or decompiled game code into the repository.**
- Do not modify the game's files. Replacing or adding a DLL beside them is the whole mechanism.
- Do not edit `ios/`. Changes to `shared/` follow `CLAUDE.md`: code, tests, golden packets and
  documents together, with a note in `docs/04-status.md`.
- If a fact about the game matters, read it from `gamekdm.dll` (the addresses in `docs/01` tell
  you where) or observe it in the running game. Do not ship a guessed behaviour; if something
  cannot be established, leave it switched off and write it down.

## What "done" looks like

1. `shared` tests pass on Windows; golden packets match.
2. The probe receives skeleton, depth and colour from `l2k_fake_sender` through your DLL at
   30 fps, with events and status behaving as `docs/01` requires.
3. The game, started with spice2x and the replacement DLL, runs a song driven by
   `l2k_fake_sender` with no sensor error.
4. `windows/STATUS.md` says, for each item above: done or not, how it was verified, what was
   measured, and what remains open — including anything in `docs/01` §7 you resolved or
   contradicted. Corrections to `docs/01` go into that document too.

The end-to-end test with the real iPhone app needs Danny: ask him when you reach it.
