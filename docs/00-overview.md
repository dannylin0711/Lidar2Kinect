# Lidar2Kinect — overview

## Goal

Dance Evolution Arcade (Konami, KDM-2016021900) is played with an Xbox 360 Kinect. Lidar2Kinect
replaces that sensor with an iPhone or iPad: the device tracks the player with its camera and
LiDAR and presents the result as Kinect v1 data.

There are three consumers of that data:

| | Consumer | Runs on | How it gets the data |
|---|---|---|---|
| 2a | **spice2x hook** — a DLL spice2x loads with the game; it redirects the game's Kinect calls | Windows PC | network, from the Lidar2Kinect app |
| 2b | **fake Kinect** — a replacement `Kinect10.dll`; the game loads it instead of Microsoft's | Windows PC | network, from the Lidar2Kinect app |
| 1 | **iPad port of the game** (separate repository) | iPad | in-process, or network from a second device |

All three need the same thing: skeleton frames, a 320×240 depth + player index image and a
640×480 colour image, in the Kinect's conventions (`docs/01` lists exactly what the game uses).

## Parts

```
 iPhone / iPad                                   Windows PC
┌──────────────────────────────┐               ┌────────────────────────────────────┐
│ Lidar2Kinect app (ios/App)   │               │ spice2x ─ game (gamekdm.dll)       │
│  capture source (ARKit…)     │               │      │ Kinect API calls            │
│        │                     │   L2K1 / UDP  │      ▼                             │
│  L2KKit pipeline ── sender ──┼──────────────▶│ fake Kinect10.dll  or  hook DLL    │
│        │  (shared C library) │               │   (shared C library + receiver)    │
└────────┼─────────────────────┘               └────────────────────────────────────┘
         │ in-process
         ▼
   iPad game port (uses L2KKit directly)
```

- **`shared/`** — the contract, as portable C99 with no dependencies: Kinect data types, the wire
  protocol, the source→Kinect mapping, the virtual camera, joint smoothing, a synthetic test
  figure. Both sides compile the same files.
- **`ios/`** — the Swift package target `L2KKit` (capture, pipeline, sender) and the universal
  iOS / iPadOS app.
- **`windows/`** — the receiver side: fake `Kinect10.dll` and spice2x hook. Not started; the brief
  for the agent that will build it is `windows/AGENT_PROMPT.md`.

## Decisions

1. **The translation happens on the Apple device.** The wire carries Kinect-shaped data, so the
   Windows side only unpacks it into the SDK's structs, and the iPad port uses the same code
   with no network in between.
2. **UDP on the local network, latest frame wins.** Dance judgement depends on timing; a late
   frame is worth less than the next one. The receiver's address is typed in or found by Bonjour.
3. **The layer imitates a physical Kinect**, tilt included (`docs/03` §1).
4. **2b before 2a on Windows.** The game imports `Kinect10.dll` statically, so without
   Microsoft's runtime installed the game does not even load; a replacement DLL solves that and
   needs nothing from spice2x. The hook (2a) is then the same core packaged differently, for
   machines that keep the real runtime. Both are in the Windows brief.
5. **"Fake Kinect" means a replacement `Kinect10.dll`,** not a virtual USB device. It covers the
   part of the SDK this game uses first; other Kinect v1 programs would need more of the API.

## The iPad port and its screens

Planned in the port's own repository (`~/code/danceevolution-ipad`), using this package:

- **External monitor connected:** the monitor shows the game; the iPad stands under or beside it
  with its rear camera and LiDAR facing the player. This is the layout the sensor is built for.
- **No monitor:** the game is on the iPad's own screen, so the rear camera faces away from the
  player. The sensor then has to be either the front camera (no LiDAR; needs the Vision-based
  source, `docs/03` §7) or a second device running the Lidar2Kinect app and streaming to the
  iPad. Which of the two is a decision for when the port reaches that point.

## Where things stand

See `docs/04-status.md`.
