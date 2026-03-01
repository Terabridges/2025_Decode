# Turret CRServo Controller — Tuning Notes

> **Branch:** `feat/turret-crservo-controller`  
> **Last updated:** 2026-02-28  
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

### PID (tuning in progress)
```
kP = 0.002      // CRServo = power→velocity, so gains must be very small
kI = 0.0
kD = 0.0005
iZone = 10.0
maxIntegral = 0.3
dAlpha = 0.4      // derivative low-pass filter (0–1; 1 = no filter)
```

**Why so small?** CRServos convert power→velocity (not torque). At kV=0.00068,
1° error × kP=0.002 = 0.002 power → ~3°/s approach velocity. Previous kP=0.015
produced 0.15 power for 10° error (→ 220°/s overshoot, violent oscillation).

### Motion Profile Constraints
```
maxProfileVelocity     = 150.0   // deg/sec (was 300 → too much momentum)
maxProfileAcceleration = 300.0   // deg/sec² (was 600 → CRServo can't track)
```

### Output Limits
```
maxOutputPower = 0.30   // prevents full-power runaway; tunable via dashboard
```
At 150°/s, feedforward needs ~0.186 power, leaving headroom for PID correction.

### On-Target Thresholds
```
onTargetPositionDeg  = 2.0   // degrees
onTargetVelocityDeg  = 10.0  // deg/sec
```

---

## 6. Bugs Fixed (Chronological)

| Commit | Issue | Root Cause | Fix |
|--------|-------|------------|-----|
| `f30223b` | eBrake oscillation | Simple reverse power created bounce | Proportional eBrake |
| `449818e` | eBrake bounce + settle tolerance | 0.3 brake power + 200°/s scale too aggressive | Power 0.20, fullVel 300, velocity-reversal release, settle threshold 10° |
| `74d795e` | REPOSITIONING stall | Target in attenuation zone | Fixed target handling |
| `c750d5c` | Characterizer corruption | Power range, settle, data corruption | Guard + settle + range fix |
| `6c7a0ec` | REPOSITIONING stall from stiction | Proportional ramp dropped to 0.048 (below stiction ~0.08) | repositionPower 0.15, ramp 10°, floor 0.10 |
| `3c77894` | SETTLING dead zone (10–15° error) | Too far to settle, not far enough to reposition | If !posOk → REPOSITIONING. Power range 0.08–0.14 |
| `6d42072` | SETTLING/REPOSITIONING oscillation (20° amplitude, 0.3s period) | Zero-power SETTLING → coast past target | Merged settle into REPOSITIONING with active proportional control |
| `840e637` | CCW characterizer steps truncated | `isNearSoftLimit()` direction-agnostic | Added `isNearSoftLimit(boolean positiveDirection)` overload |
| `87ad3df` | Placeholder feedforward values | Not yet measured | Plugged measured kV/kS values |
| (session) | Violent oscillation on TurretProfileTuner | Controller default target=180°, turret at 60° → full power from first loop | `start()` holds at current position |
| (session) | Still oscillating at ±0.30 power | kP=0.015 way too high for CRServo (power→velocity) | kP 0.015→0.002, kD 0.003→0.0005, dAlpha 0.8→0.4 |
| (session) | Turret stuck 30° from target after profile | Profile done (desVel=0) → no kS → PID alone below stiction | Added kS compensation when profile done but not on-target |
| (session) | 36° overshoot, 265°/s | maxVel=300 too fast, maxAccel=600 too aggressive | maxVel 300→150, maxAccel 600→300, maxOutputPower=0.30 |

---

## 7. Rlog Analysis Tools

### Pulling rlogs from robot
```powershell
adb pull /sdcard/FIRST/data/ C:\code\TeraBridges\2025_Decode\build\psikitReplayOut\
```

### TurretRlogDump.java test methods
- `dumpProfileTunerData()` — TurretProfileTuner: pos, vel, power, error, ff, pid, desired profile state
- `dumpStepResults()` — TurretVelocityCharacterizer step data (power → velocity)
- `dumpStictionData()` — TurretStictionCharacterizer breakaway events (position, direction, power)
- `dumpSoftLimitDiagnostics()` — Soft limit instrumentation
- `listAllKeys()` — List all logged keys in an rlog (scans ALL cycles)

### Workflow
1. Run OpMode on robot
2. `adb pull /sdcard/FIRST/PsiKit/<filename>.rlog` to local
3. Set `RLOG_PATH` env var
4. Run: `gradlew :core:cleanTest :core:test --tests "test.TurretRlogDump.<method>" --no-daemon`
5. Read output from HTML report: `core/build/reports/tests/test/classes/test.TurretRlogDump.html`
6. Also view in AdvantageScope for graphical analysis

---

## 8. Next Steps

### Profile Tuning Progress
- **Feedforward is working** — kS + kV correctly drives the turret during profile moves
- **All moves settle to ≤2° final error** and transition PROFILE → HOLD
- **Peak tracking error ~20°** during moves (turret lags behind profile)
- **Move times:** 25° move ≈ 1.1s, 120° move ≈ 1.9s, 140° move ≈ 2.0s
- **Max velocity reached:** ~190°/s (at maxProfileVelocity=150, overshoots slightly)

### Next Tuning Steps
1. **Increase kP** (0.002 → 0.003) to reduce ~20° tracking lag
2. **Increase maxOutputPower** (0.30 → 0.35) for more PID headroom
3. Consider kA adjustment for acceleration-phase tracking
4. May need to lower maxProfileVelocity if overshoot persists

### Future Work
- **Turret pivot offset:** `turretPivotOffsetX/Y` in TurretAimController (for odometry)
- **Vision integration:** `VisionTurretLockTester.java` for limelight tracking
- **TeleOp integration:** Verify turret works in full match OpMode

---

## 9. Key Conventions

- **Positive direction = CW = increasing turret angle** (after servoPowerInverted negation)
- **All callers use natural convention:** positive power → increasing angle
- **Hardware layer handles negation** — `writeServos()` negates if `servoPowerInverted`
- **Both servos receive identical power** — geared together, same direction
- **Logger prefix:** `Turret/Hardware/` and `Turret/Controller/`
- **@Configurable annotation** on both classes — fields tunable via dashboard
