## Electrical Angle Clamp Debug Plan

This guide expands on issue **“Electrical angle is forcibly clipped to 0° when running backwards”** from `docs/sin_mode_stutter_analysis.md`. The goal is to confirm the clamp is active on your hardware, capture the evidence, and implement a safe correction.

### Failure Mechanism Recap

- File: `Src/BLDC_controller.c`, block `F01_05_Electrical_Angle_Estimation`.
- After reconstructing the electrical angle (`rtb_Merge_m`), the code zeroes it whenever the interpolated value is negative:

```
if (!(rtb_Merge_m > 0)) {
  rtb_Merge_m = 0;
}
```

- In reverse motion (`rtDW->Switch2_e == -1`) the interpolator intentionally swings negative so the sine tables can advance backward. The clamp crushes those values to 0°, so the `SIN` modulator reuses the same phase vector for most of the cycle. Result: torque collapses and the motor lurches only when spinning backward.

### Instrumentation Checklist

1. **Expose the raw interpolated angle**
   - Add a debug export next to the clamp so you can view it via telemetry or a spare DAC:
     ```
     rtDW->dbg_angle_q15 = rtb_Merge_m;
     ```
   - Optionally publish it through an existing output such as `rtY->a_elecAngle_dbg`.

2. **Capture the hall-based direction flag**
   - Log `rtDW->Switch2_e` alongside the angle. If it flips to `-1` while the angle reading sticks near zero, the clamp is implicated.

3. **Check the merged angle before clamping**
   - Temporarily duplicate the variable:
     ```
     int16_T rtb_Merge_beforeClamp = rtb_Merge_m;
     ```
   - Stream both `before` and `after` values to your debugger or tracing UART. Expect to see the `before` trace go negative during reverse while the `after` trace flat-lines at zero.

4. **Timeline capture**
   - Rotate the wheel slowly backward while recording Hall codes, `Switch2_e`, `rtb_Merge_beforeClamp`, and `rtb_Merge_m`. A repeating sawtooth clipped at zero confirms the bug.

### Verifying With In-Code Debugging

Add a guarded block (surrounded with `#if defined(DEBUG_SIN_CLAMP)` to keep production binaries lean):

```
#if defined(DEBUG_SIN_CLAMP)
if (rtDW->Switch2_e == -1) {
  scope_log(0, rtb_Merge_beforeClamp);
  scope_log(1, rtb_Merge_m);
  scope_log(2, rtDW->z_counterRawPrev);
}
#endif
```

- `scope_log` can be any existing trace hook (UART, CAN, or the MDK logic analyzer). Capturing `z_counterRawPrev` helps correlate the clamp with the commutation interval.

### Potential Corrections

1. **Wrap negative angles instead of clamping**
   ```
   while (rtb_Merge_m < 0) {
     rtb_Merge_m += 23040;   // 360° in q14.2 scaling
   }
   rtb_Merge_m %= 23040;
   ```
   - Preserves symmetry—reverse motion maps to the correct sector.

2. **Apply a single addition before lookup**
   ```
   if (rtb_Merge_m < 0) {
     rtb_Merge_m += 23040;
   }
   ```
   - Works if the upstream math already limits the positive side.

3. **Delay any bounds enforcement until after `Switch2_e` multiplies in the direction**
   - Move the clamp/wrap downstream so interpolation happens in signed space, but the sine table still receives a `[0, 23040)` value.

4. **Unit-test the wrap helper**
   - Add a regression test (host-side or SIL) feeding synthetic angles: `{-5000, -1, 0, 1, 23039, 24000}`. Ensure the helper outputs continuous values when direction toggles.

### Confirmation Steps After Fix

- Re-run the reverse SIN test with the debug traces still enabled.
- Expect `rtb_Merge_m` to sweep 23040→0 smoothly without flat zones.
- Observe phase voltages on a scope: the sinusoid should now lead/lag smoothly instead of dwelling near phase-A.

Once confirmed, remove the debug guards or leave them behind a build flag for future regressions.
