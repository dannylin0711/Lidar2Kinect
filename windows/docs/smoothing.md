# NuiTransformSmooth vs l2k_smoother_apply

Open point 2 of docs/01 §7: does `shared/`'s `l2k_smoother_apply` (the Holt double-exponential
filter of Microsoft's joint-smoothing white paper) give what the game gets from ordinal 20?

## Method

`l2k_smoothcmp` (windows/tools/smoothcmp.cpp) builds 300 frames of the synthetic two-player figure
(`l2k_synth_body` → `l2k_mapper_map` → `NUI_SKELETON_FRAME`), passes each frame to Microsoft's
filter and to `l2k_smoother_apply`, and compares every joint of every tracked skeleton.

Microsoft's flat `NuiTransformSmooth` refuses to work without a sensor (`E_NUI_NOTCONNECTED`,
frame untouched; `kinect10-exports.md`). The tool therefore calls the DLL's internal routine
that the sensor method forwards to, with its own filter state — only for Kinect10.dll
1.8.0.595, which it recognises by the RVA of ordinal 20.

Cases: the game's parameters (0.5, 0.5, 0.5, 0.05, 0.04) on clean motion, with ±1 cm noise, with
±5 cm noise and the whole body jumping 0.3 m every 1.5 s (exercises the jitter and max-deviation
clamps); all-zero parameters; heavy smoothing (0.7, 0.3, 1.0, 1.0, 1.0).

## Result (2026-11-10)

```
game (0.5 0.5 0.5 0.05 0.04), clean  |ref - l2k| mean 0.00 mm, max 0.00 mm   |ref - raw| mean 15.69 mm
game, 1 cm noise                     |ref - l2k| mean 0.00 mm, max 0.00 mm   |ref - raw| mean 16.85 mm
game, 5 cm noise + 0.3 m jumps       |ref - l2k| mean 0.00 mm, max 0.00 mm   |ref - raw| mean 30.81 mm
smoothing 0                          |ref - l2k| mean 0.00 mm, max 0.00 mm   |ref - raw| mean 0.00 mm
heavy (0.7 0.3 1.0 1.0 1.0)          |ref - l2k| mean 0.00 mm, max 0.00 mm   |ref - raw| mean 123.91 mm
```

The largest per-axis difference in the CSV dump is 1e-5 m, the CSV's rounding step: the two
filters agree to float precision, while the filter itself moves the joints by 16–124 mm on
average. **`l2k_smoother_apply` reproduces the DLL's filter; `shared/` needs no change.**

Not covered: a skeleton slot that changes tracking id while staying tracked (the DLL's routine
resets a slot's history only when the slot is not tracked; `l2k_smoother` also resets on a new
tracking id — the sender frees a slot for at least one frame when a body is lost, so the two
should not diverge in practice, but it is not measured).

Run it again: `build\windows\bin\Release\l2k_smoothcmp.exe [-csv out.csv]`.
