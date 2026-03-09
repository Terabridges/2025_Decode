# Turret Yaw Calibration Runbook

This runbook uses `TurretYawCalibrationTest` to validate and calibrate:
- commanded turret angle
- raw encoder angle
- mapped encoder turret angle
- turret forward reference (`Turret.turretForwardDeg`)
- Limelight yaw feed consistency

## Scope

Use this when you see large `Vision.robotYawOffsetDeg` compensation, unstable MT2 heading alignment, or disagreement between turret command and measured angle.

Mechanical forward is the source of truth for turret forward.

## Preconditions

- Robot on blocks or in a safe area.
- Turret mechanically intact and free to move through expected range.
- OpMode deployed with `TurretYawCalibrationTest` available.
- Limelight connected if you also want yaw-feed checks.

## Opmode

- Name: `TurretYawCalibrationTest`
- Group: `Test`
- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tests/TurretYawCalibrationTest.java`

## Controls

- `Dpad Right / Left`: nudge turret target by `manualStepDeg` (default 1°)
- `Right Bumper / Left Bumper`: nudge turret target by `manualFastStepDeg` (default 5°)
- `A`: capture mechanical forward snapshot
- `X`: capture mapped minimum snapshot
- `Y`: capture mapped maximum snapshot
- `B`: sync commanded angle to measured encoder-mapped angle

## Step-by-step Procedure

1. Start `TurretYawCalibrationTest` and keep turret lock/aim lock disabled (the opmode already does this).
2. Place turret at true mechanical forward (fixture/visual reference).
3. Press `A` to capture forward.
4. Sweep across safe travel limits and press `X` at one limit and `Y` at the opposite limit.
5. Return to mechanical forward and inspect telemetry/logs:
   - `Turret cmd`
   - `Turret enc mapped`
   - `Mapped-Cmd (deg)`
   - `Mapped relative (deg)`
   - `Suggested turretForwardDeg`
6. If command/measured diverge badly, press `B` once and repeat the sweep/capture.
7. If Limelight is active, verify:
   - `Vision turret rel yaw` is near 0° at mechanical forward
   - `Vision yaw sent` tracks heading changes without large extra offset compensation

## Acceptance Criteria

- At mechanical forward, `Mapped relative (deg)` is near zero.
  - Target: within ±3°
  - Warning: above ±5°
- `Mapped-Cmd (deg)` remains small over the working range.
  - Target: within ±5°
  - Warning: above ±8°
- Captured min/max are plausible and ordered (suggested min < suggested max, with realistic travel span).

## What to Update

Use captured/suggested values to update these configurable constants:

- `Turret.turretForwardDeg`
- `Turret.turretMinDeg`
- `Turret.turretMaxDeg`
- If needed for mapping quality: `encoderRefDeg`, `encoderRefTurretDeg`, `encoderToTurretScale`, `encoderDirectionInverted`

After updates, rerun this runbook once and confirm criteria pass before tuning Limelight yaw offsets.

## Limelight Yaw Follow-up

Only after turret forward/mapping are validated:

1. Keep turret at mechanical forward.
2. Check `Vision/LimelightYawFeed/TurretRelativeYawDeg` is near zero.
3. Tune `Vision.robotYawOffsetDeg` minimally, if still required.

If large `robotYawOffsetDeg` is still needed after turret calibration, investigate chassis heading frame/sign/base (`ftcRotatedFrameBaseDeg`, `robotYawSign`).

## PsiKit Log Keys

Primary keys emitted by the opmode are under `TurretCal/*`, including:

- `TurretCal/CmdDeg`
- `TurretCal/EncoderRawDeg`
- `TurretCal/EncoderMappedDeg`
- `TurretCal/MappedMinusCmdDeg`
- `TurretCal/CmdRelativeDeg`
- `TurretCal/MappedRelativeDeg`
- `TurretCal/SuggestedForwardDeg`
- `TurretCal/SuggestedMinDeg`
- `TurretCal/SuggestedMaxDeg`
- `TurretCal/Vision/*`
