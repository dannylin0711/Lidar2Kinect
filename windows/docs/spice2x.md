# spice2x and the Kinect

Read from the spice2x source at commit `3e3db7a` (2026-10-31, github.com/spice2x/spice2x.github.io),
paths relative to `src/spice2x/`, and checked against Danny's setup (spice2x 2026-10-30,
`F:\BEMANI\DEA\KDM-2016021900\contents`).

## How the game is loaded

- The AVS game DLL of Dance Evolution is **`arkkdm.dll`**, not `gamekdm.dll`
  (`launcher/launcher.cpp:2016-2020`: `check_dll("arkkdm.dll")` sets the DEA module).
- The DEA module (`games/dea/dea.cpp`, 26 lines) loads **`gamekdm.dll` itself** with a plain
  `LoadLibraryW(MODULE_PATH / "gamekdm.dll")`, "since this DLL isn't automatically loaded the
  hooks don't work unless we load it manually". It then tries `LoadLibrary("Kinect10.dll")` only
  to warn "Failed to load 'Kinect10.dll'. Game will boot in Test Mode. Have you installed the
  Kinect SDK?" and frees it again. **That is all spice2x does about the Kinect for this game:**
  no hooks, no patches, no Nui references under `games/dea/`.
- The only Kinect code in spice2x is in FutureTomTom (`games/ftt/ftt.cpp`): it IAT-hooks ordinals
  5 and 6 of `Kinect10.dll` and patches `INuiSensor` vtable entries — a precedent for 2a, not
  used for this game.

## Which Kinect10.dll the game gets

`MODULE_PATH` is the folder of `spice.exe` (launcher.cpp:193). spice2x calls `SetDllDirectoryW`
only with `-modules` or when it falls back to a `modules` subfolder; it never calls
`AddDllDirectory` / `SetDefaultDllDirectories` and never changes the current directory. So
`gamekdm.dll`'s static import of `Kinect10.dll` is resolved by the standard search order: **the
application folder (`contents\`, where `spice.exe` is) first**, then `SysWOW64`.

**Observed:** with Microsoft's runtime installed in SysWOW64 *and* the replacement in
`contents\`, the game used the replacement (its log appeared next to it and recorded the game's
calls; 2026-11-10).

## When hook DLLs load

From `launcher/launcher.cpp`:

1. `-z` hooks: loaded before the game module (before AVS, before `gamekdm.dll`).
2. AVS core / ea3, `arkkdm.dll` (`avs::game::load_dll`).
3. `game->attach()`: the DEA module loads `gamekdm.dll` — its imports, `Kinect10.dll` included,
   are bound here and its DllMain runs.
4. `-k` hooks: `LoadLibrary(hook)` (launcher.cpp:2811-2825).
5. The game's `dll_entry_init` / `dll_entry_main` — the game's own code, Kinect thread included.

A `-k` DLL therefore runs after `gamekdm.dll`'s import table is bound but before any game code
calls the Kinect. spice2x exports no IAT helper to hooks (its `detour::iat*` is internal); its
SDK's `hook_library` only redirects later `LoadLibrary` / `GetProcAddress` calls, which cannot
change imports that are already bound. `l2k_spicehook.dll` patches the IAT itself.

## Danny's setup, observed

- `spice.exe` relaunches itself with administrator rights (UAC prompt) on every start.
- The game folder lacked `dev\raw\log`; AVS then fails to create its default logger and the game
  stops with `EXCEPTION_BREAKPOINT` in `libavs-win32.dll` before `gamekdm.dll` is loaded. Creating
  the empty folder fixed it (unrelated to the Kinect, but it was the first thing in the way).
- No DEA options are set in `%APPDATA%\spicetools.xml`; the game was started with `spice.exe -w`
  (windowed). It boots into the operator CLOCK screen first ("CLOCK NOT SET").
