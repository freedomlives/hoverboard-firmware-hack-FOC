## SIN Mode Reverse-Direction Stutter Analysis

This note summarizes several code-level mechanisms that can make the inverter behave correctly in one direction while stuttering and producing less torque in the other whenever `SIN` control is selected (`CTRL_TYP_SEL = SIN_CTRL`). Each item ties the observed physics back to the exact code path so you can inspect or instrument it further.

### 1. Electrical angle is forcibly clipped to 0° when running backwards

The sinusoidal modulator relies on `rtb_Merge_m` to hold the mechanical/electrical angle reconstructed from the Hall sensors. When direction is detected as negative (`rtDW->Switch2_e = -1`), the interpolation term becomes negative, but the result is instantly clamped at zero before it is fed to the sine tables:

```c
if (rtDW->Switch2_e == 1) {
  rtb_Sum2_h = rtConstP.vec_hallToPos_Value[Sum];
} else {
  rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[Sum] + 1);
}
rtb_Merge_m = (int16_T)(((int16_T)((int16_T)
                  ((rtb_Merge_m << 14) / rtDW->z_counterRawPrev)
                  * rtDW->Switch2_e) + (rtb_Sum2_h << 14)) >> 2);
...
if (!(rtb_Merge_m > 0)) {
  rtb_Merge_m = 0;
}
```

Source: `Src/BLDC_controller.c`, `F01_05_Electrical_Angle_Estimation`.

Physics impact: instead of sweeping the stator field smoothly from 360° back toward 300° when reversing, the code collapses any negative intermediate value back to 0°. The three-phase sine lookup that follows (`plook_u8s16_evencka`) therefore keeps commanding the stator vector near the phase-A axis for a sizable portion of each electrical revolution. The rotor keeps trying to move backwards, but it repeatedly encounters a torque vector that is ~60° out of phase, so it jerks and produces much less average torque. This effect does not show up when spinning forward because the interpolation term stays positive.

Recommended checks:

- Log `rtb_Merge_m` or the published `a_elecAngle` (`rtY->a_elecAngle`) while commanding reverse motion in SIN mode; you should see it stick near zero just before each Hall transition.
- Removing the clamp or wrapping the negative value back to `360°` (add `+23040` before calling `plook`) should restore symmetry.

### 2. Direction-detection assumes a specific Hall state sequence

Direction is inferred entirely from the order of Hall codes (`vec_hallToPos`) using:

```c
rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[Sum]
                      - rtDW->UnitDelay2_DSTATE_b);
if ((rtb_Sum2_h == 1) || (rtb_Sum2_h == -5)) {
  rtDW->Switch2_e = 1;
} else {
  rtDW->Switch2_e = -1;
}
```

Source: `Src/BLDC_controller.c`, block `F01_03_Direction_Detection`.

The lookup table baked into the model fixes the assumed Hall order:

```c
const int8_T vec_hallToPos_Value[8] = { 0, 2, 0, 1, 4, 3, 5, 0 };
```

Source: `Src/BLDC_controller_data.c`.

Physics impact: the `SIN` implementation never directly inverts the phase voltages; it rotates the stator vector by advancing or retarding the electrical angle via `Switch2_e`. If the actual wiring order produces deltas of `+2` or `-4` (common when the Hall harness is rearranged), the condition above never returns `+1`/`-5`, so `Switch2_e` stays at `-1` even while you drive forward. In forward motion the error is hidden because COM mode ignores `Switch2_e`, but in reverse the interpolated angle and the optional phase-advance block both move the sine wave the wrong way. That manifests as a proper spin in one direction and a lurching, low-torque motion in the other.

Recommended checks:

- Record the raw Hall codes as you manually rotate the wheel backward; verify the sequence matches what the table expects (`001→010→011→...`).
- If your sensor order differs (for example, wired for the "alternate board" in the README), update `vec_hallToPos_Value` or reorder the Hall leads so that successive codes always differ by `+1` when rotating forward.

### 3. Phase-advance / field-weakening term wraps differently by direction

When `FIELD_WEAK_ENA` (or phase advance in SIN mode) is enabled, the electrical angle is adjusted by the signed phase advance before being wrapped back into `[0,360)`:

```c
if (rtP->b_fieldWeakEna) {
  DataTypeConversion2 = (int16_T)((int16_T)
      ((int16_T)(rtDW->Divide3 * rtDW->Switch2_e) << 2) + rtb_Merge_m);
  DataTypeConversion2 -= (int16_T)((int16_T)(
      (int16_T)div_nde_s32_floor(DataTypeConversion2, 23040) * 360) << 6);
} else {
  DataTypeConversion2 = rtb_Merge_m;
}
```

Source: `Src/BLDC_controller.c`, block `Switch_PhaAdv`.

`rtDW->Divide3` itself comes from the (unsigned) field-weakening ramp:

```c
rtDW->Divide3 = (int16_T)((rtb_Saturation1 * rtb_Divide14_e) >> 15);
```

Source: `Src/BLDC_controller.c`, block `F04_Field_Weakening`.

Physics impact: command magnitudes above `FIELD_WEAK_LO` cause `rtDW->Divide3` to become non-zero. The advance is applied as `±phase` depending on `Switch2_e`. When moving forward the addition stays within the `[0,360)` span, so only the intended advance occurs. When moving backward, the `<< 2` scaling and subsequent subtraction land slightly below zero, so `div_nde_s32_floor` subtracts a full `360°` more than intended. The resulting discontinuity each time the advance is recomputed makes the sine phase jump by an extra electrical revolution, which looks like a periodic torque reversal only in the reverse direction.

Recommended checks:

- Temporarily set `PHASE_ADV_MAX` to zero in `Inc/config.h` and retest SIN reverse. If the symptom disappears, instrument `DataTypeConversion2` before and after the modulo to confirm the wrap-around glitch.
- When phase advance is really needed, switch the wrapping logic to add `23040` manually while `DataTypeConversion2 < 0` instead of relying on `div_nde_s32_floor`, which is sensitive to signed overflows near the wrap limit.

### 4. No PWM margin in SIN mode starves one side of the bridge

In `bldc.c`, the PWM duty sent to the gate driver is always `CLAMP(DC + pwm_res/2, pwm_margin, pwm_res - pwm_margin)`. The margin is only enabled for FOC (`pwm_margin = 110`); for `SIN`/`COM` it is set to zero:

```c
if (rtP_Left.z_ctrlTypSel == FOC_CTRL) {
  pwm_margin = 110;
} else {
  pwm_margin = 0;
}
LEFT_TIM->LEFT_TIM_U = (uint16_t)CLAMP(ul + pwm_res / 2,
                                       pwm_margin,
                                       pwm_res - pwm_margin);
```

Source: `Src/bldc.c`.

Physics impact: sinusoidal modulation demands a little off-time on every phase so the bootstrap capacitors on the high-side gate drivers can recharge. With `pwm_margin = 0` the reverse-direction sine requires one phase to be driven almost 100% high while another is almost 0%, so the high-side bootstrap never refreshes and the gate collapses mid-period. The collapse happens on whichever phase needs the prolonged high state, which depends on current direction; hence only one direction stutters and exhibits lower bus utilization. COM mode never stresses the bootstrap because it enforces well-defined 120° conduction states.

Recommended checks:

- Force `pwm_margin` to the same non-zero value you use for FOC (≈110 counts, i.e., ~5% duty) even when `SIN` is active, reflash, and observe whether reverse torque improves.
- If the bootstrap hypothesis is correct you will also see the affected phase voltage plateau near `Vbat/2` on an oscilloscope whenever the motor starts to jerk in reverse.

---

These four mechanisms are independent; more than one can be active on a real board. Instrumenting the intermediate variables called out above (angle, `Switch2_e`, phase-advance sum, PWM compare values) will quickly reveal which explanation matches your hardware. Once the suspect path is confirmed, the fix is usually small: ensure the electrical angle wraps correctly in both directions, update the Hall map to your wiring, disable or correct the signed phase-advance, or keep a modest PWM margin so the gate drivers remain biased even under high reverse duty.
