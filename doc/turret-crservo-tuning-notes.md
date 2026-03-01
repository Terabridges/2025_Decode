# Turret CRServo Controller — Tuning Notes

> **Branch:** `feat/turret-crservo-controller`  
> **Last updated:** 2026-03-01 (session 3 — high-speed profile tuning, iterations 10–29)  
> **Hardware:** Dual Axon Max CRServos (`turretL`, `turretR`) + AbsoluteAnalogEncoder (AnalogInput 0–3.3V → 0–360°)

---

## 1. Key Files

| File | Purpose |
|------|---------|
| `config/subsystems/Outtake/TurretHardware.java` | Hardware abstraction: dual CRServos, analog encoder, soft limits, eBrake |
| `config/subsystems/Outtake/TurretController.java` | PID + feedforward controller with trapezoidal motion profiling |
| `config/subsystems/Outtake/Turret.java` | High-level turret (owns Hardware + Controller) |
| `config/subsystems/Outtake/TurretAimController.java` | Shot lead / vision-aim logic |
| `config/utility/TrapezoidalMotionProfile.java` | Trapezoidal motion profile generator |
| `opmodes/tests/TurretVelocityCharacterizer.java` | Automated kV measurement (power-step sequence) |
| `opmodes/tests/TurretStictionCharacterizer.java` | Ramp-based kS (stiction) measurement |
| `opmodes/tests/TurretProfileTuner.java` | Closed-loop profile tuning OpMode |
| `PsiKit/core/src/test/java/test/TurretRlogDump.java` | Test utility: dump rlog data as CSV for analysis |

All paths relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

---

## 2. Hardware Configuration (Confirmed)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `servoPowerInverted` | `true` | Positive CRServo power **decreases** turret angle — hardware layer negates so callers use natural convention (positive = increasing angle) |
| `encoderRefDeg` | 201.8° | Raw encoder reference |
| `encoderRefTurretDeg` | 163.0° | True turret angle at reference |
| `encoderToTurretScale` | 1.030 | Turret-degrees per encoder-degree |
| `encoderDirectionInverted` | `false` | |
| Physical turret range | ~63° to ~321° (hard stops) | |
| `physicalMinDeg` / `physicalMaxDeg` | 15° / 295° | Encoder-wrap artifact clamping |

---

## 3. Soft Limits & Emergency Brake

| Parameter | Value | Notes |
|-----------|-------|-------|
| `softLimitMinDeg` | 50° | |
| `softLimitMaxDeg` | 260° | |
| `softLimitMarginDeg` | 25° | Attenuation ramp within margin |
| `emergencyBrakePower` | 0.20 | Max reverse power |
| `emergencyBrakeFullVelDps` | 300°/s | Full brake at this speed; scales linearly below |
| `emergencyBrakeVelThreshold` | 40°/s | Triggers when speed toward limit exceeds this |

eBrake has **velocity-reversal release** — releases brake when turret velocity flips sign (prevents bounce). Tracks `prevVelocityDegPerSec` to detect reversal.

`isNearSoftLimit(boolean positiveDirection)` overload ensures direction-aware soft limit checks (commit 840e637).

---

## 4. Measured Characterization Values

### Stiction (kS) — via TurretStictionCharacterizer
- **17 breakaway events** across full turret range
- Stiction is **uniform across angles** — no position-dependent hot spots

| Direction | kS | Measurements | Range | Method |
|-----------|----|-------------|-------|--------|
| CW (positive) | **0.084** | 9 points (72°–235°) | 0.082–0.090 | ramp-based breakaway |
| CCW (negative) | **0.072** | 7 points (85°–249°) | 0.071–0.073 | ramp-based breakaway |

### Velocity (kV) — via TurretVelocityCharacterizer

| Direction | kV | Clean Steps | Notes |
|-----------|----|-------------|-------|
| CW | 0.000595 | Steps 7–10 (second pass) | 0.08→0.7, 0.092→32.4, 0.104→42.1, 0.116→61.7 °/s |
| CCW | 0.000767 | 3 clean + 3 truncated | All 6 steps had data |
| **Averaged** | **0.00068** | | Used in controller |

### Max Measured Velocities
- Characterizer measured up to ~160°/s at the highest power steps
- Theoretical max from kV: ~1470°/s (but CRServo gear ratio limits actual max)

---

## 5. Current Controller Gains (TurretController.java)

> **Updated 2026-03-01 session 3 (iteration 29)** — reflects all changes
> including PID-only output filter, desired-velocity kS fade, peakVel
> normalisation, asymmetric accel/decel, and distance-dependent accel scaling.

### Feedforward (MEASURED — committed)
```
kS_CW  = 0.084    // stiction, CW direction
kS_CCW = 0.072    // stiction, CCW direction
kV     = 0.00068  // velocity feedforward (power per deg/sec)
kA     = 0.0001   // acceleration feedforward (power per deg/sec²)
```

Direction selection logic:
- `updateProfile()`: uses `desired.velocity > 0` to pick kS_CW vs kS_CCW
- `updateHold()`: uses `error > 0` to pick kS_CW vs kS_CCW

### PID
```
kP     = 0.003
kI     = 0.0
kD     = 0.001
iZone  = 10.0
maxIntegral = 0.3
dAlpha = 0.35     // derivative low-pass filter (0–1; 1 = no filter)
```

**Why so small?** CRServos convert power→velocity (not torque). At kV=0.00068,
1° error × kP=0.003 = 0.003 power → ~4°/s approach velocity. Previous kP=0.015
produced 0.15 power for 10° error (→ 220°/s overshoot, violent oscillation).

### kS Feedforward Fading

Two mechanisms control kS application to prevent overshoot and jitter:

#### During Profile Moves — Desired-Velocity Ratio Fade
```
kSVelRatioZero = 1.15
```
`velRatio = |desiredVelocity| / profile.getPeakVelocity()`
`kSScale = clamp(1 - velRatio / kSVelRatioZero, 0, 1)`

At `kSVelRatioZero=1.15`:
- Standing still (ratio=0): **full kS** — need stiction to start moving
- At cruise speed (ratio=1.0): **15% kS** — mild kinetic friction compensation
- At 1.15× peak velocity: **zero kS** — turret is fast enough

**Key design decisions (session 3):**
- Uses **desired velocity** (from profile), not actual velocity — perfectly smooth
  trapezoidal signal, no encoder noise, no filtering needed, no phase lag.
- Uses **profile.getPeakVelocity()** as denominator, not maxProfileVelocity —
  ensures short triangular moves (where peak << maxVel) get same kS fade
  behaviour at their "cruise" points as long trapezoidal moves.
- Reduced from 1.3 to 1.15 to cut CW braking impediment — kS_CW (0.084) >
  kS_CCW (0.072) means residual kS at cruise fights CW braking more.

#### Post-Profile & HOLD — Stall-Velocity Gate
```
kSStallVelDps = 50.0
```
`stallScale = clamp(1 - |vel| / kSStallVelDps, 0, 1)`

kS only applied when turret is nearly stalled (below 50°/s). Fades linearly:
- 0°/s: full kS
- 25°/s: half kS
- 50°/s: zero kS

### Motion Profile Constraints
```
maxProfileVelocity     = 400.0   // deg/sec
maxProfileAcceleration = 600.0   // deg/sec² (accel phase, long moves)
maxProfileDeceleration = 450.0   // deg/sec² (decel phase)
```

**Asymmetric accel/decel (session 3):** `TrapezoidalMotionProfile` now supports
different rates for accel vs decel phases. The turret accelerates fine at 600+
but can't physically brake faster than ~450°/s². With asymmetric rates, a 140°
move reaches ~268°/s peak but decelerates at the proven safe rate.

### Short-Move Acceleration Scaling
```
shortMoveMinAccel     = 350.0   // deg/sec² for very short moves
shortMoveMinAccelDeg  =  15.0   // distance at or below which minAccel is used
shortMoveFullAccelDeg =  60.0   // distance at or above which full accel is used
```

For moves shorter than 60°, acceleration is linearly blended between 350 and 600°/s².
This prevents the CRServo from overshooting the velocity profile on short distances
where the accel phase is only a few frames long. At 20° (our typical short move),
effective accel ≈ 372°/s².

**Iteration 29 result:** Short-move overshoot dropped from 12–15° to 0.1–5.2°.

### Output Limits & Filtering
```
maxOutputPower      = 0.50
outputFilterAlpha   = 0.3    // PID-only output filter
```

**PID-only output filter (session 3):** Only the PID component is smoothed —
feedforward passes through unfiltered for instant response to velocity changes.
`output = lastFeedforward + pidFiltered` where `pidFiltered = α·rawPID + (1-α)·prev`.
Bypassed in DIRECT mode. Logged as `Output` (final) vs `RawOutput` (pre-filter).

### On-Target Thresholds
```
onTargetPositionDeg  = 2.0   // degrees
onTargetVelocityDeg  = 10.0  // deg/sec
```

---

## 6. Session 2 — kS Feedforward Tuning History (2026-02-28)

This session focused on eliminating overshoot and power jerkiness caused by
kS (stiction compensation) feedforward. **~10 deploy-test-fix cycles.**

### Iteration 1: Baseline (start of session)
- **Problem:** Violent oscillation on first TurretProfileTuner run
- **Root cause:** Controller default target=180° vs actual position 60° → full power from first loop
- **Fix:** `controller.setTargetAngleImmediate(hardware.getPositionDeg())` in `start()`
- Added `maxOutputPower` clamp (0.30)
- Reduced PID: kP 0.015→0.002, kD 0.003→0.0005, dAlpha 0.8→0.4
- **Commit:** `0143b22` — "fix: profile tuner oscillation + stiction dead zone + gain tuning"

### Iteration 2: Increase kP for tighter tracking
- kP 0.002→0.003, maxOutputPower 0.30→0.35
- **Result:** Turret running 13-15° **ahead** of profile during cruise, overshooting ~11° past target
- **Root cause:** kS always applied in forward direction even when turret ahead of profile

### Iteration 3: kP=0.005 attempt
- Oscillations got bad — limit cycle with kS
- **Rolled back** to kP=0.003, increased kD 0.0005→0.001, dAlpha 0.4→0.6

### Iteration 4: Conditional kS (hard threshold)
- Added `turretAhead` check: suppress kS when turret >2° ahead of profile
- **Result:** Overshoot improved dramatically (19°→2-8°)
- **Problem:** ±0.084 power swing at 2° boundary caused bang-bang toggling

### Iteration 5: Smooth linear kS position fade
- Replaced hard cutoff with linear fade over `kSFadeoutDeg=5°`
- **Result:** Slightly better but still jerky — position-based fade had ~200ms lag

### Iteration 6: Velocity-based kS suppression
- Added `kSVelFadeoutDps=30`: suppress kS when actual velocity exceeds desired by 30°/s
- **Result:** Still jerky — 30°/s fade window too narrow for turret's 80°/s velocity swings

### Iteration 7: Velocity-ratio kS fade
- Replaced position+velocity fade with single velocity-ratio approach
- `kSVelRatioZero=1.5` (full kS at 50% of desVel, 50% at cruise, zero at 150%)
- **Result:** Bad — Move 7 had 47 jerks, maxVel=331°/s
- **Root causes:** (a) 50% kS at cruise = +0.042 power = +62°/s extra, (b) post-profile full kS direction-switching = limit cycle

### Iteration 8: Stall-gated kS (deployed, analyzed)
- `kSVelRatioZero` 1.5→1.0 (zero kS at cruise velocity)
- Added `kSStallVelDps=20`: post-profile/HOLD kS only applied when |vel| < 20°/s
- `updateHold` signature changed to include `currentVelDegPerSec` parameter
- **Result:** "Turret motion is overall smoother, commanded power still a little jerky"
- **Rlog analysis (5 moves):**

| Move | From→To | Dist | Time | Settle | FinalErr | MaxErr | Overshoot | MaxVel | MaxPwrSwing | Jerks |
|------|---------|------|------|--------|----------|--------|-----------|--------|-------------|-------|
| 1 | 220→100 | 120° | 5.17s | 4.35s | 0.7° | 13.3° | **0.0°** | 259 | 0.164 | 27 |
| 2 | 219→80  | 139° | 2.62s | 2.60s | 1.7° | 11.7° | **0.0°** | 169 | 0.088 | 13 |
| 3 | 81→220  | 139° | 3.21s | 1.77s | 0.7° | 18.8° | **0.0°** | 302 | 0.170 | 25 |
| 4 | 219→80  | 139° | 5.70s | 4.51s | 0.7° | 15.7° | **0.0°** | 306 | 0.192 | 40 |
| 5 | 219→100 | 119° | 3.97s | 2.61s | 0.9° | 10.6° | **0.0°** | 204 | 0.113 | 20 |

Key finding: **ZERO overshoot across all moves** (massive improvement from 19° earlier!).
But jerk counts 13-40 and settle times up to 4.5s.

#### Move 4 Detailed Analysis (worst case, 40 jerks)
Two jerk sources identified:

**Phase A — Cruise under-drive:** With `kSVelRatioZero=1.0`, kS=0 at cruise.
Turret needs `kS + kV·vel = 0.072 + 0.102 = 0.174` to hold 150°/s. Only 0.102
delivered → PID compensates with ~10° tracking error → velocity oscillates 96-173°/s.

**Phase B — Post-profile stall-gate limit cycle (last 12°):**
1. vel slow (<20) → kS ON (+0.072 power) → turret accelerates
2. vel fast (>20) → kS OFF (-0.072 power swing) → turret decelerates  
3. Repeat every ~200ms, crawling 1-2° per cycle → 4.5s to settle 12°

### Iteration 9: Output filter + kS tuning
Based on Move 4 analysis, 4 changes:

| Parameter | Before | After | Why |
|-----------|--------|-------|-----|
| `kSVelRatioZero` | 1.0 | **2.0** | Full kS at cruise. Turret needs stiction compensation to maintain 150°/s. Only fades when turret significantly exceeds desired speed. |
| `kSStallVelDps` | 20 | **50** | Wider post-profile fade band. Reduce limit-cycle amplitude by spreading toggle over 2.5× more velocity range. |
| `dAlpha` | 0.6 | **0.35** | More derivative filtering. Noise amplified by kD was contributing PID jitter. |
| `outputFilterAlpha` | *(new)* | **0.3** | Output low-pass filter (EMA). Directly smooths all remaining power jitter. ~120ms to 95% of step at 50Hz. Bypassed in DIRECT mode. |

Also added `RawOutput` logging (pre-filter) for comparison with `Output` (post-filter).

---

## 7. Session 3 — High-Speed Profile Tuning (2026-02-28 → 2026-03-01)

This session focused on increasing turret speed from 120°/s to 280+ °/s while
maintaining low overshoot. **~20 deploy-test-fix cycles (iterations 10–29).**
Major architectural changes: PID-only output filter, desired-velocity kS fade,
peakVel normalisation, asymmetric accel/decel, and distance-dependent accel scaling.

### Architectural Breakthroughs

#### Iteration 13: PID-only output filter
- **Problem:** Iteration 9's full output filter caused positive-feedback oscillation —
  filter smoothed the COMBINED FF+PID output, so when FF changed direction the
  filter delayed the response, PID overcorrected, filter delayed correction, etc.
- **Fix:** Split output filter — FF passes through unfiltered, only PID is filtered.
  `output = lastFeedforward + pidFiltered`
- **Result:** Velocity tracking better (peak velocities dropped from 296-307 to
  186-218°/s), but power jerkier (mxPS 0.076-0.136) because unfiltered FF now
  exposed raw encoder velocity noise in the kS fade signal.

#### Iteration 16: Desired-velocity kS fade (KEY BREAKTHROUGH)
- **Problem:** kS fade used `|actualVelocity| / |desiredVelocity|` — actual velocity
  from encoder is noisy, and any filtering introduces phase lag.
- **Fix:** Changed ratio to `|desiredVelocity| / maxProfileVelocity`. Desired velocity
  comes from the trapezoidal profile — a perfectly smooth signal. No noise, no
  filtering needed, no phase lag.
- **Result:** BEST YET — CW velocity overshoot solved (max 144°/s vs 80%+ before),
  avg jerks 9.6, near-zero overshoot on long moves.

#### Iteration 19: peakVel normalisation
- **Problem:** Short triangular moves (e.g. 80→100, 20°) at 250°/s maxVel — the
  profile's peak velocity was only ~100°/s but kS ratio used maxProfileVelocity
  (250). So ratio stayed low (0.4) → near-full kS throughout the move → 21° overshoot.
- **Fix:** Changed ratio denominator to `profile.getPeakVelocity()` instead of
  `maxProfileVelocity`. Short moves now fade kS correctly at their actual cruise.
- **Result:** Short CW overshoot dropped from 21° to 3-12°.

### Speed Progression

| Iter | maxVel | maxAccel | maxDecel | kSVelRatio | maxPwr | Key Change | Result |
|------|--------|----------|----------|-----------|--------|------------|--------|
| 9 | 150 | 350 | — | 2.0 | 0.35 | Output filter + kS tuning | Baseline |
| 10 | 150 | 350 | — | 2.0 | 0.35 | Deploy iter 9 | Smoother but still jerky |
| 11 | 150 | 350 | — | 2.0 | 0.35 | Filter + diagnostic refinement | Incremental |
| 12 | 150 | 350 | — | 2.0 | 0.35 | Full output filter investigations | Positive-feedback oscillation discovered |
| **13** | 120 | 350 | — | 1.3 | 0.35 | **PID-only output filter** | Velocity tracking improved |
| 14 | 120 | 350 | — | 1.3 | 0.35 | +kSVelFilterAlpha (vel smoothing) | **Worse** — smoothing lag |
| 15 | 120 | 350 | — | 1.0 | 0.35 | Zero kS at cruise | Velocity oscillation |
| **16** | 120 | 350 | — | 1.3 | 0.35 | **Desired-vel kS fade** | Breakthrough — smooth + accurate |
| 17 | 150 | 350 | — | 1.3 | 0.35 | Speed up | Near-perfect, avg jerks 5.6 |
| 18 | 200 | 350 | — | 1.3 | 0.35 | Speed up | Short CW 21° overshoot |
| **19** | 250 | 350 | — | 1.3 | 0.35 | **peakVel normalisation** | Short CW overshoot 3-12° |
| 20 | 300 | 350 | — | 1.3 | 0.35 | Speed up | Hit maxOutputPower=0.35 clamp |
| 21 | 350 | 350 | — | 1.3 | 0.50 | maxPwr 0.35→0.50 | Vel still ~215°/s (accel limited) |
| 22 | 350 | 600 | — | 1.3 | 0.50 | Accel 350→600 | Overshoot 4.6-18.3° (can't brake) |
| 23 | 350 | 450 | — | 1.3 | 0.50 | Accel 600→450 | CCW good (0-1.8°), CW still 3.5-18.5° |
| 24 | 350 | 450 | — | **1.15** | 0.50 | **kSVelRatio 1.3→1.15** | CW long 2.5-5.7°, CCW 0° — big improvement |
| 25 | 400 | 800 | — | 1.15 | 0.50 | Accel 450→800 | 8-27° overshoot everywhere |
| 26 | 400 | 600 | — | 1.15 | 0.50 | Accel 800→600 | 6-17° overshoot still too high |
| **27** | 400 | 700 | **450** | 1.15 | 0.50 | **Asymmetric accel/decel** | Long 2-6°, short 6-13° |
| 28 | 400 | 600 | 450 | 1.15 | 0.50 | Accel 700→600 | Long 1-7°, short 8-15° |
| **29** | 400 | 600 | 450 | 1.15 | 0.50 | **Distance-dependent accel** | Long 1-7°, **short 0.1-5°** — FINAL |

### Detailed Iteration Results

#### Iteration 23: CW/CCW asymmetry identified
- maxAccel=450, kSVelRatioZero=1.3
- CCW long moves: 0.1–1.8° overshoot (excellent)
- CW long moves: 3.5–18.5° overshoot (bad)
- **Root cause:** kS_CW=0.084 > kS_CCW=0.072 → residual 30% kS at cruise
  (from kSVelRatioZero=1.3) impedes CW braking more than CCW braking.

#### Iteration 24: kS at cruise reduced → CW overshoot fixed
- kSVelRatioZero 1.3→1.15 (30%→15% kS at cruise)
- **CW long moves:** 2.5–5.7° overshoot (was 3.5–18.5°)
- **CCW long moves:** 0.0° overshoot (perfect)
- **CW short moves (20°):** 4.2–6.2° (was 9-13.6°)
- Max velocities 206–255°/s, power peaking at ~0.11
- **Best iteration yet for CW behaviour**

#### Iteration 25: accel=800 — too aggressive
- Velocities hit 350–418°/s but 8–28° overshoot everywhere
- Turret simply cannot decelerate at 800°/s²

#### Iteration 26: accel=600 — still overshooting
- Long moves 6–10° OS, short moves 12–17° OS
- 600°/s² symmetric decel still too fast for physical braking

#### Iteration 27: Asymmetric accel/decel introduced
- `TrapezoidalMotionProfile` extended with separate `maxDeceleration` parameter
- Accel at 700°/s² (fast ramp-up), decel at 450°/s² (proven safe rate)
- For 140° move: peak ~280°/s (faster than iter 24's 250°/s)
- **Long moves:** 1.9–5.6° overshoot at 280–325°/s — good
- **Short moves:** 6.4–12.9° — 700°/s² accel on 20° causes velocity overshoot
  (profile peak ~105°/s, actual hits 150–160°/s)

#### Iteration 28: accel=600 — long moves good, short still overshooting
- `maxProfileAcceleration=600`, `maxProfileDeceleration=450`
- **Long moves (140°):** 0.9–6.9° overshoot at 246–266°/s — good
- **Short moves (20°):** 7.7–15.0° — CRServo overshoots the velocity profile
  despite lower accel because 600°/s² is still too aggressive for 20° distances
- M10 anomaly: 124.5° OS — likely cascading from M9's 26.8° overshoot

#### Iteration 29: Distance-dependent acceleration scaling (FINAL)
- Added `shortMoveMinAccel=350`, `shortMoveMinAccelDeg=15`, `shortMoveFullAccelDeg=60`
- Acceleration linearly blends from 350°/s² (short) to 600°/s² (long)
- For 20° move: effective accel ≈ 372°/s², peak velocity ≈ 90°/s
- **Results (15 moves):**

| Category | Moves | Overshoot | MaxVel | Jerks | Notes |
|----------|-------|-----------|--------|-------|-------|
| Long CW (120–140°) | M2,M4 | 6.2–6.9° | 241–262 | 2–5 | Good |
| Long CCW (120–140°) | M3,M5,M14 | 1.5–4.5° | 253–266 | 3–7 | Excellent |
| Short CW (20°) | M1,M8,M12 | 0.9–3.2° | 95–103 | 2–3 | **Huge improvement** |
| Short CCW (20°) | M7,M9,M11 | 0.1–1.9° | 76–88 | 1–3 | Near-perfect |
| Short CW (20°, outlier) | M6,M10 | 5.2–6.0° | 109–110 | 2–3 | kS_CW asymmetry |

**Summary:** Short-move overshoot dropped from 12–15° (iter 28) to 0.1–5.2°.
Long moves unchanged at 250–280°/s with 1.5–7° OS. This is the production tuning.

---

## 8. Bugs Fixed (All Sessions)

| Commit | Issue | Root Cause | Fix |
|--------|-------|------------|-----|
| `f30223b` | eBrake oscillation | Simple reverse power created bounce | Proportional eBrake |
| `449818e` | eBrake bounce + settle tolerance | 0.3 brake power + 200°/s scale too aggressive | Power 0.20, fullVel 300, velocity-reversal release, settle threshold 10° |
| `74d795e` | REPOSITIONING stall | Target in attenuation zone | Fixed target handling |
| `c750d5c` | Characterizer corruption | Power range, settle, data corruption | Guard + settle + range fix |
| `6c7a0ec` | REPOSITIONING stall from stiction | Proportional ramp dropped to 0.048 (below stiction ~0.08) | repositionPower 0.15, ramp 10°, floor 0.10 |
| `3c77894` | SETTLING dead zone (10–15° error) | Too far to settle, not far enough to reposition | If !posOk → REPOSITIONING. Power range 0.08–0.14 |
| `6d42072` | SETTLING/REPOSITIONING oscillation | Zero-power SETTLING → coast past target | Merged settle into REPOSITIONING with active control |
| `840e637` | CCW characterizer steps truncated | `isNearSoftLimit()` direction-agnostic | Added `isNearSoftLimit(boolean dir)` overload |
| `87ad3df` | Placeholder feedforward values | Not yet measured | Plugged measured kV/kS values |
| `0143b22` | Violent oscillation + stiction dead zone | Default target≠actual; gains too high; PID alone below stiction | Hold at current; reduce gains; add kS when profile done |
| *(uncommitted)* | 19° overshoot from kS | kS always pushed forward even when ahead | Velocity-ratio kS fade |
| *(uncommitted)* | Post-profile kS limit cycle | Full kS toggling ±0.072 at 20°/s threshold | Stall-velocity gate (kSStallVelDps=50) |
| *(uncommitted)* | Power jitter during cruise | kS zero at cruise (kSVelRatioZero=1.0) | Increased to 2.0, later refined to 1.15 |
| *(uncommitted)* | Power jitter from derivative noise | dAlpha=0.6 too responsive | Decreased to 0.35 |
| *(uncommitted)* | Output filter positive feedback | Full output filter delayed FF direction change | PID-only output filter (iter 13) |
| *(uncommitted)* | Encoder noise in kS fade | Actual velocity noisy → kS fade jittery | Desired-velocity kS fade (iter 16) |
| *(uncommitted)* | Short-move kS overshoot | kS ratio used maxProfileVel, low for short moves | peakVel normalisation (iter 19) |
| *(uncommitted)* | CW braking asymmetry | kS_CW > kS_CCW, 30% kS at cruise impedes CW brake | kSVelRatioZero 1.3→1.15 (iter 24) |
| *(uncommitted)* | Symmetric decel too fast | Turret can't brake at 600+°/s² | Asymmetric accel/decel in profile (iter 27) |
| *(uncommitted)* | Short-move velocity overshoot | 600°/s² accel too aggressive for 20° moves | Distance-dependent accel scaling (iter 29) |

---

## 9. Rlog Analysis Tools

### Pulling rlogs from robot
```powershell
# Latest file:
adb shell "ls -lt /sdcard/FIRST/PsiKit/ | head -3"
# Pull:
adb pull /sdcard/FIRST/PsiKit/<filename>.rlog C:\code\TeraBridges\2025_Decode\build\psikitReplayOut\
```

### TurretRlogDump.java test methods
- `dumpProfileTunerData()` — pos, vel, power, error, ff, pid, desired profile state, rawPower
- `dumpStepResults()` — TurretVelocityCharacterizer step data
- `dumpStictionData()` — stiction breakaway events
- `dumpSoftLimitDiagnostics()` — soft limit instrumentation
- `listAllKeys()` — all logged keys in an rlog

### Workflow
1. Run OpMode on robot
2. `adb pull /sdcard/FIRST/PsiKit/<filename>.rlog` to local
3. Set `RLOG_PATH` env var:
   ```powershell
   $env:RLOG_PATH = "C:\code\TeraBridges\2025_Decode\build\psikitReplayOut\<filename>.rlog"
   ```
4. Run: `cd C:\code\TeraBridges\PsiKit; .\gradlew.bat :core:cleanTest :core:test --tests "test.TurretRlogDump.dumpProfileTunerData" --no-daemon`
5. Read output from HTML report: `core/build/reports/tests/test/classes/test.TurretRlogDump.html`
6. Also view in AdvantageScope for graphical analysis

### PowerShell Move Metrics Script
After dumping CSV, analyze moves with this pattern:
```powershell
# Parse CSV, detect move starts (HOLD→PROFILE transitions), compute per-move:
# - distance, time, settle time, final error, max error, overshoot
# - max velocity, max power swing (frame-to-frame), jerk count (swings > 0.03)
```
(Paste the full script from rlog analysis history)

---

## 10. CRServo Control Theory Notes

### Why CRServo tuning is different from DC motor tuning
- **DC motor:** power → torque → acceleration. PID on position is standard.
- **CRServo:** power → velocity (internal speed controller). The servo's internal
  controller handles torque. External PID on position produces *velocity commands*.
  This means:
  - **kP must be tiny** — 1° error should produce ~0.003 power (→ ~4°/s approach),
    not 0.15 power (→ 220°/s overshoot)
  - **Momentum/overshoot is the primary problem** — turret momentum carries past target
    even after power cut. The servo's internal controller quickly reaches commanded
    speed but can't reverse instantly.
  - **kS (stiction) is dominant** — the dead zone from 0 to ~0.08 power produces
    no motion. Without FF, PID alone can't produce enough power for small errors.
  - **kV model is approximate** — the internal servo controller's response isn't
    perfectly linear, so kV only roughly predicts velocity from power.

### kS Feedforward Strategy (current — iteration 24+)
The kS feedforward has gone through ~20 iterations. The final approach uses:

1. **Profile active, desVel ≠ 0:** Desired-velocity ratio fade.
   `ratio = |desiredVelocity| / profile.getPeakVelocity()`.
   kS fades linearly from full at ratio=0 to zero at ratio=kSVelRatioZero (1.15).
   At cruise (ratio=1.0), 15% kS remains for mild kinetic friction compensation.
   Uses the profile's desired velocity (perfectly smooth, no noise) and peakVelocity
   (correct normalisation for short triangular moves).

2. **Profile finished / HOLD, error > 1°:** Stall-velocity gate. kS applied only
   when turret is nearly stopped (<50°/s). If moving, kS fades linearly to zero.
   This prevents the kS direction-switching limit cycle.

3. **PID-only output filter (alpha=0.3):** Only PID component is smoothed — FF
   passes through unfiltered for instant response. Catches PID jitter without
   the positive-feedback problem that full output filtering caused.

### Asymmetric Accel/Decel
The turret's CRServos can produce high acceleration (the internal motor controller
responds quickly to velocity increases) but deceleration is limited by the servo's
ability to brake against momentum. With 1:1 gear ratio, the turret has significant
inertia relative to servo braking torque. Empirically:
- Acceleration: 600–700°/s² is achievable
- Deceleration: 450°/s² is the maximum the turret can physically track without
  overshooting. Higher rates cause the profile to demand a stop position while
  the turret is still carrying velocity.

---

## 11. Next Steps

### Tuning complete
Iteration 29 represents the production tuning. All major control issues resolved:
- Long moves: 250–280°/s with 1.5–7° overshoot, 2–5 jerks
- Short moves: 0.1–5° overshoot, 1–3 jerks
- CW/CCW asymmetry managed (kSVelRatioZero=1.15)
- Power smooth (mxPS < 0.10 typically)

### Potential follow-up tuning
- **kI for steady-state:** Currently 0. Could help with cruise tracking error
  but risks overshoot
- **Gear ratio analysis:** See `doc/turret-gear-ratio-analysis.md` — the 1:1
  ratio means kS is ~75% of cruise FF, making control fundamentally challenging.
  A higher gear ratio (e.g. 3:1) would make kS negligible relative to kV·vel.

### Future integration
- **Turret pivot offset:** `turretPivotOffsetX/Y` in TurretAimController
- **Vision integration:** `VisionTurretLockTester.java` for limelight tracking
- **TeleOp integration:** Verify turret works in full match OpMode

---

## 12. Key Conventions

- **Positive direction = CW = increasing turret angle** (after servoPowerInverted negation)
- **All callers use natural convention:** positive power → increasing angle
- **Hardware layer handles negation** — `writeServos()` negates if `servoPowerInverted`
- **Both servos receive identical power** — geared together, same direction
- **Logger prefix:** `Turret/Hardware/` and `Turret/Controller/`
- **@Configurable annotation** on both classes — fields tunable via dashboard
- **Jerk metric:** count of frame-to-frame power changes > 0.03 during a move
