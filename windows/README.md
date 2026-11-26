# Windows side

The receiving end of Lidar2Kinect: makes Dance Evolution Arcade believe a Kinect v1 is attached
and feeds it what the iOS app streams. Status: `STATUS.md`. Specification: `../docs/01`.

| Folder | What |
|---|---|
| `core/` | static library: L2K1 receiver (UDP thread, JPEG via WIC, DNS-SD) and the virtual sensor (`INuiSensor`, frame textures, events, status, the flat Kinect10 functions). `nui_abi.h` declares the Kinect ABI from the documentation, with `static_assert`s on what the game depends on. `l2kw_base.h` is the part without the Kinect ABI (config, log, receiver, hub interface), also built for x64. |
| `vp4u/` | **DanceAround** — `visionposewrapper.dll`, x64, the vp4u API 210 fed by the receiver; own CMake project (`vp4u/README.md`) |
| `kinect10/` | **2b** — `Kinect10.dll`, x86, all 26 exports of Microsoft's 1.8 DLL at their real ordinals |
| `spicehook/` | **2a** — `l2k_spicehook.dll`, loaded with spice2x `-k`; redirects `gamekdm.dll`'s Kinect10 imports to the core |
| `probe/` | `l2k_probe.exe` — drives Kinect10.dll exactly like `gamekdm.dll` (ordinal imports, events, states); the regression test |
| `tools/` | `l2k_smoothcmp.exe` — compares `l2k_smoother_apply` with Microsoft's filter |
| `tests/` | `test_core.exe` — the sensor without the network |
| `docs/` | findings: export table, spice2x, smoothing, no-sender behaviour, config and log |

## Build and test

Visual Studio 2022 (its bundled CMake is enough), 32-bit:

```
cmake -S windows -B build/windows -A Win32
cmake --build build/windows --config Release
ctest --test-dir build/windows -C Release --output-on-failure
```

Everything lands in `build/windows/bin/Release/`. The tests: `shared`'s golden packets
(`l2k`), the sensor unit test (`core`), and two probe runs against `Kinect10.dll` fed by
`l2k_fake_sender` over loopback (`probe_stream`: 30 fps of all three streams, events, contents;
`probe_reconnect`: BYE, return through the status callback, silent sender).

## Install (2b, recommended)

Copy `Kinect10.dll` (and optionally `l2k.ini`, see `docs/config.md`) into the game's `contents\`
folder, next to `spice.exe` and `gamekdm.dll`. Nothing else: no Microsoft runtime needed, no
spice2x option. Allow `spice.exe` through the Windows firewall for UDP 24810 if the app cannot
reach it. Remove the file to go back to a real Kinect.

## Install (2a)

Only with a `Kinect10.dll` that the game can load anyway (Microsoft's runtime installed). Copy
`l2k_spicehook.dll` next to `spice.exe` and start `spice.exe -k l2k_spicehook.dll`. See `STATUS.md`
for what it adds.

## Try it without an iPhone

```
build\windows\bin\Release\l2k_fake_sender.exe -n 1 -t 3600 -j shared\tests\vectors\testcard_640x480.jpg 127.0.0.1
build\windows\bin\Release\l2k_probe.exe -t 10
```

The fake sender films its figure like the cabinet's Kinect, from 2 m up tilted −24°; `-H metres`
and `-a degrees` place it elsewhere (`docs/skeleton-panel.md` explains what the game's SENSOR
CHECK makes of the placement).
