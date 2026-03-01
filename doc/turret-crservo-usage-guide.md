# CRServo Turret System — Usage Guide

> **Branch:** `feat/turret-crservo-controller`  
> **Initial commit:** `1b29d96` — CRServo turret controller with PID+FF, motion
> profiling, vision/odo tracking, and tuning OpModes  
> **Final tuning commit:** `d59f56f` — finalize CRServo tuning (iteration 29)

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Hardware Setup](#2-hardware-setup)
3. [Bring-Up Workflow](#3-bring-up-workflow)
4. [Calibration OpModes](#4-calibration-opmodes)
5. [Tuning OpModes](#5-tuning-opmodes)
6. [Integration OpModes](#6-integration-opmodes)
7. [Using the Turret in TeleOp / Auto](#7-using-the-turret-in-teleop--auto)
8. [Aim Modes Reference](#8-aim-modes-reference)
9. [Key Parameters Reference](#9-key-parameters-reference)
10. [Logging & Analysis](#10-logging--analysis)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Architecture Overview

The turret system is split into three layers, each with a single responsibility:

```
┌─────────────────────────────────────────────────────────┐
│                        Turret.java                      │
│                (wires everything together)              │
├─────────────────┬──────────────────┬────────────────────┤
│ TurretHardware  │ TurretController │ TurretAimController│
│  - CRServo ×2   │  - PID + FF      │  - Aim mode logic  │
│  - Analog enc   │  - Motion profile│  - Vision / Odo    │
│  - Soft limits  │  - Output filter │  - Shot lead       │
│  - Velocity est │  - kS fade       │  - Chassis yaw     │
└─────────────────┴──────────────────┴────────────────────┘
```

| Class | Responsibility |
|-------|----------------|
| **TurretHardware** | Reads encoder, estimates velocity (ring-buffer linear regression), writes power to both CRServos, enforces soft limits and emergency braking |
| **TurretController** | Closed-loop PID + feedforward (kS, kV, kA) with trapezoidal motion profiling. Three modes: PROFILE, HOLD, DIRECT |
| **TurretAimController** | Decides *what angle* to target each loop — manual, preset position, vision tracking, odometry tracking, or auto (vision+odo fallback) |
| **Turret** | Subsystem coordinator. Calls `hardware.update()` → `controller.update()` → `hardware.setPower()` each loop. Exposes a clean public API |
| **TrapezoidalMotionProfile** | Generates time-parameterized trapezoidal velocity profiles with asymmetric accel/decel |
| **ShotLeadCalculator** | Computes aim offset from chassis velocity and time-of-flight physics |

### Update cycle (every loop)

```java
// Inside Turret.update():
hardware.update();                              // 1. Read encoder + velocity
double power = controller.update(pos, vel);     // 2. Compute PID+FF output
hardware.setPower(power);                       // 3. Drive servos (with soft limits)
```

---

## 2. Hardware Setup

### Required hardware map entries

| Name | Type | Description |
|------|------|-------------|
| `turretL` | CRServo | Left turret CRServo |
| `turretR` | CRServo | Right turret CRServo |
| `turretAnalog` | AnalogInput | Absolute analog encoder (0–3.3V → 0–360°) |

Both CRServos receive **identical power** — they are geared together and must
turn in the same direction. The hardware layer handles servo polarity inversion
via `servoPowerInverted`.

### Polarity convention

| Convention | Direction |
|------------|-----------|
| **Positive power** in code | Turret angle **increases** |
| **Negative power** in code | Turret angle **decreases** |
| `servoPowerInverted = true` | Hardware layer negates power before writing to servos |

> All callers (OpModes, controllers) use the natural convention: positive =
> increasing angle. The `servoPowerInverted` flag handles the physical wiring.

### Safe operating range

| Parameter | Value | Description |
|-----------|-------|-------------|
| Soft limit min | 50° | Power toward min is blocked |
| Soft limit max | 260° | Power toward max is blocked |
| Margin | 25° | Power attenuated linearly within margin |
| Physical clamp | 15–295° | Encoder wrap artifacts clamped to these bounds |

---

## 3. Bring-Up Workflow

Follow these steps **in order** when setting up a new turret or after hardware
changes. Each step builds on the results of the previous one.

### Step 1: Verify servos — TurretServoSync

**Purpose:** Verify both CRServos spin, are wired correctly, and are in sync.

1. Deploy and run **TurretServoSync**
2. Use **Left Stick Y** to ramp power slowly
3. Verify both servos spin in the same direction
4. Check if `invertRight` needs to be toggled
5. Use Dpad to step power in 0.01 increments for fine control
6. Try disabling one servo at a time (`enableLeft`/`enableRight`) to verify each

### Step 2: Map the encoder — TurretEncoderCalibrator

**Purpose:** Determine the encoder-to-turret-degree mapping (reference point + scale).

1. Deploy and run **TurretEncoderCalibrator**
2. Manually position the turret to a known physical angle using LStick X
3. Set `trueAngleDeg` via Dpad or dashboard to match the physical position
4. Press **A** to capture the (encoder, true) pair
5. Repeat at 4–8 positions spread across the turret range
6. Press **Y** to compute the best-fit linear calibration
7. Copy the displayed `encoderRefDeg`, `encoderRefTurretDeg`, and
   `encoderToTurretScale` values into `TurretHardware.java`

### Step 3: Verify encoder accuracy — TurretEncoderAccuracyTest

**Purpose:** Verify the calibrated encoder mapping produces accurate position
readings.

1. Deploy and run **TurretEncoderAccuracyTest**
2. Press **A / X / Y / B** to go to preset angles (fwd, 90°, 180°, 270°)
3. Use **Dpad L/R** to step by `stepDeg` increments
4. Verify the turret reaches each target and the error is within tolerance
5. Check "Settled" indicator — turret held within tolerance for 300ms

### Step 4: Characterize stiction — TurretStictionCharacterizer

**Purpose:** Measure the minimum power to overcome static friction (kS) in each
direction.

1. Deploy and run **TurretStictionCharacterizer**
2. Position the turret near center using LStick X
3. Verify direction shows **CW (+)** — press **B** to toggle if needed
4. Press **A** to start a ramp test
5. Watch the turret — it will slowly ramp power until it moves
6. Note the **Last Breakaway CW** value
7. Press **B** to switch to CCW, then **A** to ramp again
8. Note the **Last Breakaway CCW** value
9. Press **Y** to capture the CW/CCW pair at this position
10. Repeat at 3–4 different positions to check for variation
11. Copy `kS_CW` and `kS_CCW` into `TurretController.java`

### Step 5: Characterize kV — TurretVelocityCharacterizer

**Purpose:** Measure the power-to-velocity relationship to determine kV.

1. Deploy and run **TurretVelocityCharacterizer**
2. Set `startPower` just above kS (e.g., 0.08)
3. Set `endPower` to a moderate level (e.g., 0.14)
4. Press **A** to start the automated sequence
5. The turret will:
   - Reposition to the start side
   - Settle and pause
   - Run at each power level for `steadyStateDurationSec`
   - Measure average velocity in the last `measureWindowSec`
6. When DONE, read the results table and the computed **kV** (slope) and
   **kS** (intercept) from least-squares fit
7. Copy `kV` into `TurretController.java`
8. Press **Y** to toggle direction and repeat for CCW validation

### Step 6: Tune PID — TurretPIDAutoTuner (optional)

**Purpose:** Get initial PID gains via relay feedback (Åström–Hägglund method).

1. Deploy and run **TurretPIDAutoTuner**
2. Set `relayPower` to a moderate value (e.g., 0.20)
3. Press **A** to start relay oscillation
4. Wait for the required number of oscillation cycles (default 6)
5. Read the suggested kP, kI, kD values
6. Press **Y** to toggle between Ziegler-Nichols and Tyreus-Luyben tuning rules

> **Note:** For CRServo turrets, PID gains must be very small (kP ≈ 0.003).
> The auto-tuner gives a starting point — manual fine-tuning using the profile
> tuner is usually needed.

### Step 7: Tune motion profiles — TurretProfileTuner

**Purpose:** Tune the complete PID+FF+profile pipeline with real moves.

1. Deploy and run **TurretProfileTuner**
2. Press **A** to go to 100°, **B** for 220°, **X** for 80°
3. Watch the move metrics: overshoot, settle time, peak velocity, tracking error
4. Tune via dashboard (@Configurable):
   - **maxProfileVelocity** — cruise speed target
   - **maxProfileAcceleration** — how fast to speed up
   - **maxProfileDeceleration** — how fast to slow down
   - **kP, kD** — PID tracking gains
   - **kSVelRatioZero** — when kS fades to zero (lower = less kS at cruise)
   - **maxOutputPower** — safety clamp
   - **outputFilterAlpha** — PID smoothing (lower = smoother, slower)
5. Iterate: make a change, press a target button, observe results
6. Test both long moves (120–140°) and short moves (20°)

### Step 8: Test vision tracking — VisionTurretLockTester

**Purpose:** Verify the turret tracks an AprilTag with the full aim-lock pipeline.

1. Deploy and run **VisionTurretLockTester**
2. Press **A** to enable aim lock
3. Drive the robot with the left stick while the turret auto-tracks
4. Verify the turret follows the target smoothly
5. Press **X** for blue alliance tag (20), **Y** for red (24)
6. Dpad L/R for manual nudge adjustments

---

## 4. Calibration OpModes

### TurretEncoderCalibrator

| | |
|---|---|
| **Driver Station name** | `TurretEncoderCalibrator` (Test group) |
| **Purpose** | Multi-point least-squares encoder calibration |
| **Output** | `encoderRefDeg`, `encoderRefTurretDeg`, `encoderToTurretScale`, R² |

**Controls:**

| Input | Action |
|-------|--------|
| LStick X | Nudge turret for positioning |
| Dpad L/R | Adjust `trueAngleDeg` ± small step |
| Dpad U/D | Adjust `trueAngleDeg` ± large step |
| A | Capture (encoder, true) pair |
| Y | Compute best-fit calibration |
| X | Clear all captures |
| B | Stop servo (safety) |

### TurretStictionCharacterizer

| | |
|---|---|
| **Driver Station name** | `TurretStictionCharacterizer` (Test group) |
| **Purpose** | Automated stiction (kS) measurement via power ramp |
| **Output** | `kS_CW`, `kS_CCW`, per-position breakaway table |

**Controls:**

| Input | Action |
|-------|--------|
| LStick X | Manual nudge (idle only) |
| A | Start ramp test |
| B | Toggle CW / CCW direction |
| Y | Capture CW+CCW breakaway pair |
| X | Clear all captures |
| BACK | Abort ramp (safety) |

### TurretVelocityCharacterizer

| | |
|---|---|
| **Driver Station name** | `TurretVelocityCharacterizer` (Test group) |
| **Purpose** | Multi-step power-vs-velocity measurement for kV |
| **Output** | kV (slope), kS (intercept), velocity table |

**Controls:**

| Input | Action |
|-------|--------|
| LStick X | Manual nudge (idle only) |
| A | Start characterization sequence |
| B | Abort and stop |
| Y | Toggle CW / CCW direction |

**Phases:** IDLE → REPOSITIONING → PAUSING → RUNNING → (repeat) → DONE

---

## 5. Tuning OpModes

### TurretProfileTuner

| | |
|---|---|
| **Driver Station name** | `TurretProfileTuner` (Test group) |
| **Purpose** | Full PID+FF+profile tuning with move metrics |
| **Output** | Overshoot, settle time, peak velocity, tracking error per move |

**Controls:**

| Input | Action |
|-------|--------|
| A | Go to Target A (100°) |
| B | Go to Target B (220°) |
| X | Go to Target C (80°) |
| Y | Go to panel target (configurable) |
| Dpad Up | Go to center (180°) |
| LStick X | Manual nudge |
| BACK | Stop and hold current position |

**Key @Configurable parameters** (tune via FTC Dashboard):
- All `TurretController` fields (kP, kD, kV, kS, profile constraints, etc.)
- `targetA`, `targetB`, `targetC`, `panelTarget` on the OpMode itself

### TurretPIDAutoTuner

| | |
|---|---|
| **Driver Station name** | `TurretPIDAutoTuner` (Test group) |
| **Purpose** | Relay feedback PID auto-tuning (Åström–Hägglund) |
| **Output** | Suggested kP, kI, kD via Ziegler-Nichols or Tyreus-Luyben |

**Controls:**

| Input | Action |
|-------|--------|
| A | Start relay oscillation |
| B | Abort and stop |
| Y | Toggle Z-N / T-L tuning rule |
| X | Set relay target to current position |
| Dpad U/D | Adjust relay power ±0.01 |
| LStick X | Manual nudge (idle only) |

---

## 6. Integration OpModes

### VisionTurretLockTester

| | |
|---|---|
| **Driver Station name** | `VisionTurretLockTester` (Test group) |
| **Purpose** | Vision-based aim lock with chassis-yaw assist |
| **Requires** | Full Robot (including vision/Limelight) |

**Controls (gamepad1):**

| Input | Action |
|-------|--------|
| A | Toggle aim lock on/off |
| X | Switch to tag 20 (blue goal) |
| Y | Switch to tag 24 (red goal) |
| B | Reset turret to init |
| Dpad L/R | Manual nudge |
| LStick | Mecanum translation |
| RStick X | Mecanum yaw (+ chassis yaw assist) |

### TurretEncoderAccuracyTest

| | |
|---|---|
| **Driver Station name** | `TurretEncoderAccuracyTest` (Test group) |
| **Purpose** | Verify position accuracy at commanded angles |

### TurretFrameCharacterization

| | |
|---|---|
| **Driver Station name** | `TurretFrameCharacterization` (Test group) |
| **Purpose** | Characterize encoder behavior, direction, and tracking |

### TurretPositionCalibrator

| | |
|---|---|
| **Driver Station name** | `TurretPositionCalibrator` (Test group) |
| **Purpose** | Capture forward direction, min/max limits, goal bearings |

### TurretServoSync

| | |
|---|---|
| **Driver Station name** | `TurretServoSync` (Test group) |
| **Purpose** | Raw servo wiring and sync verification (no PID) |

### TurretTest

| | |
|---|---|
| **Driver Station name** | `TurretTest` (Test group) |
| **Purpose** | Minimal smoke test — set angle from dashboard |

---

## 7. Using the Turret in TeleOp / Auto

### Basic setup

```java
// In your OpMode init:
Robot robot = new Robot(hardwareMap, telemetry);

// The turret is part of the Outtake subsystem:
Turret turret = robot.outtake.turret;
```

### Profiled move to a position

```java
// Command a move with trapezoidal motion profiling
turret.setTargetAngle(120.0);   // Go to 120°

// In your loop:
robot.update();  // This calls turret.update() internally

// Check if arrived:
if (turret.isOnTarget()) {
    // At target, within 2° and velocity < 10°/s
}
```

### Immediate target (no profiling)

```java
// For vision/odo tracking where target changes every frame
turret.setTargetAngleImmediate(desiredAngle);
```

### Manual nudge (joystick)

```java
// Set nudge velocity — turret integrates this into position each loop
double joystickX = gamepad1.right_stick_x;
turret.setManualVelocity(joystickX * 180.0);  // ±180°/s max
```

### Enable aim lock (vision + odometry auto)

```java
// Enable auto aim tracking (vision-preferred with odo fallback)
turret.setAimLockEnabled(true);

// Or toggle:
turret.toggleAimLock();

// In your loop — call this to run the aim controller:
turret.updateAimLock(vision, flywheelRPM);
```

### Check vision on-target

```java
// True when vision lock is active and turret is within tolerance
if (turret.isVisionOnTarget(vision, 2.0)) {
    // Aim is locked — safe to shoot
}
```

### Chassis yaw assist

When the turret hits a travel limit but vision wants to keep turning, the aim
controller outputs a chassis yaw correction:

```java
double yawAssist = turret.getChassisYawCorrection();
// Mix into your mecanum yaw input:
double yaw = gamepad1.right_stick_x + yawAssist;
```

### Shot lead compensation

```java
turret.setShotLeadEnabled(true);

// Provide chassis velocity for lead calculation:
ShotLeadCalculator calc = turret.getShotLeadCalculator();
double leadOffset = calc.calculateLeadOffsetDeg(
    chassisVx, chassisVy, robotHeadingRad,
    turretForwardDeg, currentTurretDeg,
    distanceToTarget, flywheelRPM
);
```

---

## 8. Aim Modes Reference

| Mode | Description | Target selection | Use case |
|------|-------------|-----------------|----------|
| **MANUAL** | Holds current angle, accepts nudge velocity | Manual joystick | Default, driver control |
| **POSITION** | Profiled move to specific angle | `setTargetAngle()` | Auto paths, preset positions |
| **VISION_TRACK** | Closed-loop on Limelight tx | AprilTag detection | Dedicated vision aim |
| **ODO_TRACK** | Aim at field point via follower pose | Odometry | When vision unavailable |
| **AUTO** | Vision-preferred with odo fallback | Automatic | Match play — best general mode |

### AUTO mode fallback behavior

1. Vision available → track with vision (LockSource: TX)
2. Vision lost < 180ms → hold current position (short dropout)
3. Vision lost > 180ms → fall back to odometry (LockSource: ODO)
4. Vision reacquired → wait 140ms reacquire delay, then switch back to vision

---

## 9. Key Parameters Reference

### TurretController (PID + FF)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kP` | 0.003 | Proportional gain (very small for CRServo) |
| `kD` | 0.001 | Derivative gain |
| `kI` | 0.0 | Integral gain (unused) |
| `dAlpha` | 0.35 | Derivative low-pass filter coefficient |
| `kV` | 0.00068 | Velocity feedforward (power per °/s) |
| `kS_CW` | 0.084 | Stiction power, CW direction |
| `kS_CCW` | 0.072 | Stiction power, CCW direction |
| `kA` | 0.0001 | Acceleration feedforward |

### Motion Profile

| Parameter | Value | Description |
|-----------|-------|-------------|
| `maxProfileVelocity` | 400 °/s | Maximum cruise velocity |
| `maxProfileAcceleration` | 600 °/s² | Acceleration rate (long moves) |
| `maxProfileDeceleration` | 450 °/s² | Deceleration rate |
| `shortMoveMinAccel` | 350 °/s² | Accel for very short moves (≤15°) |
| `shortMoveMinAccelDeg` | 15° | Below this distance, use `shortMoveMinAccel` |
| `shortMoveFullAccelDeg` | 60° | Above this distance, use full accel |

### Output Shaping

| Parameter | Value | Description |
|-----------|-------|-------------|
| `maxOutputPower` | 0.50 | Maximum power output (safety clamp) |
| `outputFilterAlpha` | 0.3 | PID-only EMA filter (lower = smoother) |
| `kSVelRatioZero` | 1.15 | Velocity ratio at which kS fades to zero |
| `kSStallVelDps` | 50 °/s | Max velocity for HOLD stiction compensation |

### On-Target Thresholds

| Parameter | Value | Description |
|-----------|-------|-------------|
| `onTargetPositionDeg` | 2.0° | Position tolerance for on-target |
| `onTargetVelocityDeg` | 10.0 °/s | Velocity threshold for settled |

### TurretHardware

| Parameter | Value | Description |
|-----------|-------|-------------|
| `encoderRefDeg` | 201.8° | Encoder reading at reference position |
| `encoderRefTurretDeg` | 163.0° | True turret angle at reference position |
| `encoderToTurretScale` | 1.030 | Turret degrees per encoder degree |
| `softLimitMinDeg` | 50° | Minimum safe turret angle |
| `softLimitMaxDeg` | 260° | Maximum safe turret angle |
| `softLimitMarginDeg` | 25° | Power attenuation zone width |
| `servoPowerInverted` | true | Negate power for servo wiring polarity |
| `emergencyBrakePower` | 0.20 | Max emergency brake power |
| `emergencyBrakeFullVelDps` | 300 °/s | Velocity for full brake power |

---

## 10. Logging & Analysis

All subsystems log to **PsiKit** rlog files via `Logger.recordOutput()`.

### Log key prefixes

| Prefix | Data |
|--------|------|
| `Turret/Hardware/` | Raw encoder, mapped position, velocity, commanded power, soft limit actions |
| `Turret/Controller/` | Mode, target, error, output, feedforward, PID, integral, profile state |
| `Turret/Aim/` | Aim mode, lock source, aim target, manual velocity, filtered tx, chassis yaw |
| `Calibrator/Encoder/` | Encoder calibrator captures and fit results |
| `Calibrator/Stiction/` | Stiction ramp data and breakaway powers |

### Viewing logs

1. **AdvantageScope** — connect to rlog port 5802 for real-time viewing, or open
   `.rlog` files for replay
2. **TurretRlogDump** (in PsiKit) — extract turret data from `.rlog` files:
   ```powershell
   $env:RLOG_PATH = "path\to\file.rlog"
   .\gradlew :core:test --tests "*.TurretRlogDump" --no-daemon
   ```
3. **analyze_moves.ps1** — parse TurretRlogDump output for move statistics
   (overshoot, max velocity, jerks)

### Key signals for tuning

| Signal | What to look for |
|--------|-----------------|
| `Controller/Profile/DesiredPosDeg` | Profile setpoint — should be smooth trapezoid |
| `Hardware/MappedPositionDeg` | Actual position — should track setpoint closely |
| `Controller/Output` | Final power — should be smooth, no jitter at cruise |
| `Controller/RawOutput` | Pre-filter output — compare with Output to see filter effect |
| `Controller/Feedforward` | FF component — should dominate, with PID as small correction |
| `Hardware/VelocityDegPerSec` | Measured velocity — should follow profile velocity |
| `Hardware/SoftLimit/Action` | Should be "none" during normal operation |

---

## 11. Troubleshooting

### Turret doesn't move

1. Check servo wiring — run **TurretServoSync** to verify both servos spin
2. Check `softLimitsEnabled` — the turret may be at a limit
3. Check `servoPowerInverted` — wrong polarity means power pushes into the limit
4. Check encoder connection — if position reads NaN, the analog input isn't connected

### Violent oscillation

1. **Reduce `maxOutputPower`** — start at 0.10 and increase slowly
2. **Check `servoPowerInverted`** — wrong polarity creates positive feedback
3. **Reduce kP** — CRServo gains must be very small (0.001–0.005)
4. **Increase `outputFilterAlpha`** — heavier PID filtering (try 0.2)

### Overshooting target

1. **Long moves:** Reduce `maxProfileDeceleration` (can't brake faster than physics)
2. **Short moves:** Reduce `shortMoveMinAccel` (CRServo can't track high accel
   over short distances)
3. **Both:** Reduce `kSVelRatioZero` toward 1.0 (less residual kS at cruise)
4. **Check kS values** — re-run stiction characterizer if in doubt

### Jitter at cruise speed

1. **Increase `kSVelRatioZero`** — more aggressive kS fade reduces jitter
2. **Decrease `outputFilterAlpha`** — more PID smoothing
3. **kS fade is working correctly if** `Controller/Feedforward` is smooth at cruise

### Can't reach target speed

1. **Increase `maxOutputPower`** — the safety clamp may be too low
2. **Check kV** — at 400°/s target, kV·400 = 0.272 power is needed plus kS
3. **Check profile** — the profile may be triangular (short distance) and never
   reaching maxVelocity. This is normal for short moves.

### CW/CCW asymmetry (behaves differently in each direction)

1. Re-run **TurretStictionCharacterizer** — kS may differ per direction
2. Adjust `kSVelRatioZero` — asymmetric kS causes directional braking differences
3. The CW/CCW asymmetry is a physical characteristic of the drivetrain and cannot
   be fully eliminated; the fade compensates for it

### Emergency brake fires unexpectedly

1. Check `emergencyBrakeVelThreshold` (default 40°/s) — lower = more sensitive
2. Check soft limit positions — recalibrate if the turret's physical range changed
3. The proportional brake (`emergencyBrakePower * |vel| / fullVel`) prevents
   bounce; if the turret bounces at limits, increase `emergencyBrakeFullVelDps`

---

## Quick Reference: OpMode Cheat Sheet

| OpMode | When to use | Key controls |
|--------|-------------|--------------|
| **TurretServoSync** | First bring-up, servo verification | LStick = power, A = zero |
| **TurretEncoderCalibrator** | After hardware changes | A = capture, Y = compute fit |
| **TurretStictionCharacterizer** | After hardware changes | A = ramp, B = toggle dir, Y = capture |
| **TurretVelocityCharacterizer** | After kS changes | A = start sequence |
| **TurretPIDAutoTuner** | Initial PID gain estimation | A = start oscillation |
| **TurretProfileTuner** | Iterative tuning (most time here) | A/B/X = preset moves, dashboard = gains |
| **TurretEncoderAccuracyTest** | Verify calibration | A/B/X/Y = preset angles |
| **TurretFrameCharacterization** | Debug direction/tracking | Dpad = step, presets = 0/90/180/270 |
| **TurretPositionCalibrator** | Capture forward, limits, goals | A/B/Start/X/Y = capture labels |
| **VisionTurretLockTester** | Vision integration test | A = toggle lock, sticks = drive |
| **TurretTest** | Smoke test | A = go to dashboard angle |
