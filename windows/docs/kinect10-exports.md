# Kinect10.dll: exports and ABI

Read on 2026-11-10 from Microsoft's Kinect for Windows Runtime **1.8.0.595**, 32-bit DLL
`C:\Windows\SysWOW64\Kinect10.dll` (SHA-256 `e4119006…56bf74`, PE timestamp 2013-08-21), with
`dumpbin /exports` and `dumpbin /disasm`. Nothing of the DLL is in this repository.

## Export table

Ordinal base 1, 29 function slots, 25 names. Ordinals 2–4 are empty. "Pops" is the `ret n` of
each function in the real DLL, i.e. the size of its `__stdcall` arguments. The replacement
(`windows/kinect10/kinect10.def`) has the same table and the same stack sizes (checked by
disassembling both).

| Ord | Name | Pops | Game uses |
|---|---|---|---|
| 1 | *(no name)* — `mov eax, <static data>; cdq; ret`: no arguments, returns a pointer | 0 | |
| 5 | `NuiGetSensorCount(int *)` | 4 | **yes** |
| 6 | `NuiCreateSensorByIndex(int, INuiSensor **)` | 8 | **yes** |
| 7 | `NuiCameraElevationGetAngle(LONG *)` | 4 | |
| 8 | `NuiCameraElevationSetAngle(LONG)` | 4 | |
| 9 | `NuiImageGetColorPixelCoordinatesFromDepthPixel(res, viewArea*, x, y, USHORT depth, LONG*, LONG*)` | 28 | **yes** |
| 10 | `NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(cres, dres, viewArea*, x, y, USHORT, LONG*, LONG*)` | 32 | |
| 11 | `NuiImageStreamGetNextFrame(HANDLE, DWORD, const NUI_IMAGE_FRAME **)` | 12 | |
| 12 | `NuiImageStreamOpen(type, res, flags, limit, HANDLE event, HANDLE *)` | 24 | |
| 13 | `NuiImageStreamReleaseFrame(HANDLE, const NUI_IMAGE_FRAME *)` | 8 | |
| 14 | `NuiInitialize(DWORD)` | 4 | |
| 15 | `NuiSetFrameEndEvent(HANDLE, DWORD)` | 8 | |
| 16 | `NuiShutdown()` | 0 | |
| 17 | `NuiSkeletonGetNextFrame(DWORD, NUI_SKELETON_FRAME *)` | 8 | |
| 18 | `NuiSkeletonTrackingDisable()` | 0 | |
| 19 | `NuiSkeletonTrackingEnable(HANDLE, DWORD)` | 8 | |
| 20 | `NuiTransformSmooth(NUI_SKELETON_FRAME *, const NUI_TRANSFORM_SMOOTH_PARAMETERS *)` | 8 | **yes** |
| 21 | `NuiCreateSensorById(const OLECHAR *, INuiSensor **)` | 8 | |
| 22 | `NuiSetDeviceStatusCallback(NuiStatusProc, void *)` | 8 | **yes** |
| 23 | `NuiImageStreamSetImageFrameFlags(HANDLE, DWORD)` | 8 | |
| 24 | `NuiImageStreamGetImageFrameFlags(HANDLE, DWORD *)` | 8 | |
| 25 | `NuiGetAudioSource(INuiAudioBeam **)` | 4 | |
| 26 | `NuiSkeletonSetTrackedSkeletons(DWORD *)` | 4 | |
| 27 | `NuiSkeletonCalculateBoneOrientations(const NUI_SKELETON_DATA *, NUI_SKELETON_BONE_ORIENTATION *)` | 8 | |
| 28 | `NuiCreateCoordinateMapperFromParameters(ULONG, void *, INuiCoordinateMapper **)` | 12 | |
| 29 | `NuiCreateDepthFilter(LPCWSTR, DWORD, INuiDepthFilter **)` | 12 | |

**Correction to docs/01 §2:** ordinal 9 is `NuiImageGetColorPixelCoordinatesFromDepthPixel`
(7 arguments, 28 bytes), not the `…AtResolution` variant (ordinal 10, 8 arguments). The game's
call site (`0x10019cf5`) pushes exactly seven arguments, `(2, NULL, x, y, depth_mm << 3, &cx, &cy)`, and does not pop
them (stdcall).
The other four pairings in docs/01 are confirmed.

`gamekdm.dll`'s import table lists the ordinals in the order 5, 6, 20, 9, 22 (IAT
`0x102181d4`…`0x102181e4`).

## Struct layouts and the vtable

`windows/core/nui_abi.h` declares the structs and interfaces from the SDK documentation and
checks with `static_assert` everything the game's code reveals: `NUI_IMAGE_FRAME` = 40 bytes,
`dwFrameNumber` at +8, `pFrameTexture` at +0x14; `NUI_SKELETON_FRAME` = 0xA68 bytes,
`vFloorClipPlane` at +0x10; `NUI_SKELETON_DATA` = 436 bytes. The `INuiSensor` method order of
SDK 1.8 puts every method the game calls at the offset the game calls it (docs/01 §3: 0x0c,
0x10, 0x14, 0x18, 0x24, 0x28, 0x38–0x4c, 0x68), and `INuiFrameTexture::LockRect` at 0x14,
`UnlockRect` at 0x1c. The SDK's own headers were not installed; the layout of the methods the
game never calls (between those offsets and after 0x68) comes from the documentation only.

## Behaviour of the real DLL without a sensor

- `NuiGetSensorCount` → `S_OK`, count 0 (no Kinect on this PC).
- `NuiTransformSmooth` (flat, ordinal 20) → `0x83010014` (`E_NUI_NOTCONNECTED`), frame untouched:
  it looks up the default sensor first. The `INuiSensor` method forwards to an internal routine
  (RVA `0x3F9D0` in this build; an assertion string names it
  `_NuiTransformSmooth(m_ppSkeletonMotionFilterState, pSkeletonFrame, pSmoothingParams)`). That
  routine takes 6 pointers to per-slot filter state blocks of 0x414 bytes, the frame, and the
  parameters (in ECX); it resets a slot's state when the slot is not tracked. `l2k_smoothcmp`
  calls it directly to compare the filters — see `smoothing.md`.
