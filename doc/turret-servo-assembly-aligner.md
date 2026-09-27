# Turret Servo Assembly Aligner Guide

This guide explains how to use the `TurretServoAssemblyAligner` test op mode to set up the paired turret servos, check that they move together, and optionally compare their encoder response.

## Op mode and hardware

- Driver Station name: `TurretServoAssemblyAligner`
- Group: `Test`
- Source: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tests/TurretServoAssemblyAligner.java`
- Required configured devices: servos named `turretL` and `turretR`, and an analog input named `floodgate`.
- Optional device: an analog input named `turretAnalog`. Without it, the op mode still runs, but encoder readings and characterization samples are unavailable.

The op mode applies a PWM range of 500 to 2500 microseconds by default. Confirm that the servos support the configured range and that the turret linkage can safely reach the requested positions before moving the mechanism.

## Safety and output behavior

- Secure the robot and keep hands clear of the turret gears. Make sure the turret is free to move through the range you intend to test.
- When the op mode is idle, both servo PWM outputs are disabled. In ALIGN, the outputs also start disabled; press `Y` after starting to enable both servos. VERIFY enables both automatically. CHARACTERIZE enables only its selected servo.
- Disabling PWM removes the servo's active holding torque. Support the turret if it could move or fall when torque is removed.
- Stop immediately if a servo strains, chatters, or the gears bind. Do not force a powered servo shaft or use the offset to force incompatible gears together.
- The code permits servo positions from `minPos` to `maxPos` (defaults `0.0` to `1.0`). Those software limits are not a guarantee that the robot's mechanism can safely travel through the entire range.

## Starting and stopping

1. Select `TurretServoAssemblyAligner` on the Driver Station.
2. While idle, press `START` to cycle through `ALIGN`, `VERIFY`, and `CHARACTERIZE`. The currently selected mode is shown in telemetry. The initial selection is ALIGN.
3. Press `A` to start the selected mode.
4. To stop the test mode, press the right-stick button. This disables both PWM outputs. Stop the OpMode on the Driver Station when finished.

`START` only changes the selected mode while idle. In ALIGN, `A` also returns the target to `centerPos`, so press it again after starting if you want to recenter.

## ALIGN: set assembly position

ALIGN is intended for positioning the servos during mechanical assembly. It has a shared target position, with the right servo command optionally inverted and offset from the left command.

1. Select ALIGN and press `A` to start. Both PWM outputs remain disabled initially.
2. Adjust the shared target while outputs are disabled if needed. Then press `Y` to enable both servos. The servos move to the commands shown in `LeftCmd` and `RightCmd`.
3. Move to the intended assembly reference and fit or mesh the servo horns/gears at that position. If you need to inspect one servo by itself, `X` toggles left PWM and `B` toggles right PWM. `Y` enables both again.
4. Make only small adjustments, checking that neither servo is loaded against the other. Press the right-stick button to stop and disable PWM when assembly checks are complete.

ALIGN controls while running:

| Control | Action |
| --- | --- |
| Left stick up/down | Continuously increase/decrease `targetPos` |
| D-pad right/left | Increase/decrease `targetPos` by `smallStep` (default `0.0025`) |
| Right/left bumper | Increase/decrease `targetPos` by `largeStep` (default `0.01`) |
| `A` | Set `targetPos` to `centerPos` (default `0.50`) |
| `X` | Toggle left servo PWM |
| `B` | Toggle right servo PWM |
| `Y` | Enable both servo PWM outputs |
| D-pad up/down | Increase/decrease `rightOffset` by `0.001` |
| Right/left trigger | Increase/decrease `rightOffset` by `0.005` per press |
| `BACK` | Set `targetPos` to `centerPos` |
| Right-stick button | Stop ALIGN and disable both PWM outputs |

The D-pad up/down and trigger offset adjustments work in every running mode, although ALIGN's on-screen control hints do not mention them. Triggers are edge-triggered: release and press again for each increment.

## VERIFY: check paired motion

VERIFY enables both servos and lets you move them together manually or run a bounded automatic sweep.

1. Select VERIFY while idle and press `A` to start. Both servos are enabled.
2. With auto-sweep off, move the target slowly using the left stick, D-pad left/right, and bumpers. Check mechanical clearance and listen/feel for gear binding.
3. To test a limited repeated sweep, press `X` to toggle auto-sweep. It travels between `verifySweepMin` and `verifySweepMax` (defaults `0.35` and `0.65`) at `verifySweepRatePerSec` (default `0.20` position units per second). Press `X` again to stop it. Press `Y` to reverse the sweep direction.
4. Press `A` to stop auto-sweep and return to center. Press the right-stick button to stop the mode.

While VERIFY is running, D-pad up/down and the triggers adjust `rightOffset` using the increments listed above. Watch the turret as you make each small change; an offset changes the right servo's command relative to the left, and the mechanical result depends on how the servos and gears are installed.

## CHARACTERIZE: compare individual servo response

CHARACTERIZE moves one servo at a time and records command/encoder samples. Use this mode only if `turretAnalog` is configured and its reading is useful over the test range.

1. Select CHARACTERIZE and press `A` to start. The left servo is initially selected and enabled; the right servo PWM is disabled.
2. Move to a target position and press `A` to capture a sample for the selected servo. Telemetry shows the active servo and sample count.
3. Press `Y` to switch the active servo. Repeat the same commanded positions for the other servo so the data covers comparable command values. The active servo is enabled and the other servo is disabled after switching.
4. Press `X` to compare the left and right samples. Check `Matched Points`, average encoder delta, maximum encoder delta, and the suggested right-offset adjustment. Samples are matched by command within `compareCommandTolerance` (default `0.01`), so too few matched points means the two data sets do not have enough overlapping command values.
5. Press `B` to clear the samples for the currently selected servo only. To stop CHARACTERIZE, press the right-stick button.

CHARACTERIZE controls:

| Control | Action |
| --- | --- |
| Left stick, D-pad left/right, bumpers | Adjust shared target as in ALIGN |
| `A` | Capture a sample for the active servo |
| `Y` | Switch active servo between LEFT and RIGHT |
| `X` | Compare the collected left and right sample sets |
| `B` | Clear samples for the active servo |
| `BACK` | Return target to `centerPos` |
| D-pad up/down, right/left trigger | Adjust `rightOffset` |
| Right-stick button | Stop mode and disable both PWM outputs |

The default sample cap is 20 per servo. Captures require the optional turret encoder; otherwise `Turret Enc Deg` reports `not available` and samples are not recorded.

## Understanding the offset and telemetry

The right command is calculated from the shared target and the offset:

- If `invertRight` is false (the default), `RightCmd = targetPos + rightOffset`.
- If `invertRight` is true, `RightCmd = 1 - targetPos + rightOffset`.
- Both commands are clamped to the configured `minPos`/`maxPos` range, and the target is restricted to the shared range where both commands can fit.

The default `rightOffset` is `0.021`, so the two command values are not necessarily equal even at one shared target. Check `LeftCmd`, `RightCmd`, `Invert Right`, and `Right Offset` before interpreting the mechanical alignment. `rightOffset` is a servo-position adjustment, not an angle in degrees or a direct measurement of gear tooth error.

Useful Driver Station telemetry includes:

- `Selected Mode`, `Mode Running`, `TargetPos`: current mode state and shared target.
- `LeftCmd`, `RightCmd`, `Left Enabled`, `Right Enabled`: requested servo positions and whether PWM is enabled.
- `L getPosition`, `R getPosition`: SDK-reported servo positions; these are not physical position feedback.
- `L PWM us`, `R PWM us`: estimated pulse widths from the configured PWM range.
- `Turret Enc Deg`: analog encoder angle if available.
- `FloodgateAmps`: current estimate from the required `floodgate` analog input.
- CHARACTERIZE-only telemetry: sample counts, matched points, encoder errors, and suggested offset adjustment.

## Tuning values and keeping results

The main constants are `@Configurable` static fields in `TurretServoAssemblyAligner.java`. Defaults include `centerPos = 0.50`, `rightOffset = 0.021`, `invertRight = false`, target steps of `0.0025` and `0.01`, and VERIFY sweep endpoints of `0.35` and `0.65`.

The controller adjustments to `rightOffset` change the in-memory value for the current Robot Controller session; they do not edit the Java source. Record the final telemetry value and update the `rightOffset` initializer in the source if the calibration should be reproducible after a restart or redeploy. Update `invertRight` in the source/configuration only if the right servo must run in the opposite direction; verify that change carefully before operating the coupled mechanism.