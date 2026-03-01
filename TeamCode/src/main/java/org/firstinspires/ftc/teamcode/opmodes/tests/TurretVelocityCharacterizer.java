package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretHardware;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

/**
 * Automated velocity characterizer for the CRServo turret.
 *
 * <h3>Purpose</h3>
 * Determines the velocity feedforward gain {@code kV} by commanding constant power
 * levels (above kS) and measuring steady-state velocity. Also validates that
 * velocity is roughly linear with power.
 *
 * <h3>How It Works</h3>
 * <ol>
 *   <li>Configure the power levels to test via {@code startPower}, {@code endPower},
 *       and {@code powerSteps}.</li>
 *   <li>Press A to begin the automated sequence.</li>
 *   <li>For each power level, the turret runs for {@code steadyStateDurationSec}
 *       and the last {@code measureWindowSec} of velocity data is averaged.</li>
 *   <li>After all steps, displays a table and computes kV via least-squares.</li>
 *   <li>Press B to abort at any time.</li>
 * </ol>
 *
 * <h3>Controls</h3>
 * <ul>
 *   <li><b>A</b> — start/restart the characterization sequence</li>
 *   <li><b>B</b> — abort and stop</li>
 *   <li><b>Y</b> — toggle direction (CW ↔ CCW)</li>
 *   <li><b>Left Stick X</b> — manual nudge when idle</li>
 * </ul>
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretVelocityCharacterizer", group = "Test")
public class TurretVelocityCharacterizer extends OpMode {
    private static final String LOG_PREFIX = "Calibrator/Velocity/";
    private static final int MAX_STEPS = 20;

    //---------------- Configurable ----------------
    /** Starting power level (must be above CRServo stiction, ~0.08). */
    public static double startPower = 0.08;
    /** Ending power level. Keep low enough that the turret can sustain steady-state
     *  across the full travel without hitting soft limits (~0.14 → ~80°/s × 1.5s = 120°). */
    public static double endPower = 0.14;
    /** Number of power steps to test. */
    public static int powerSteps = 6;
    /** Duration at each power level to reach steady state (sec). */
    public static double steadyStateDurationSec = 1.5;
    /** Window at end of each step to measure average velocity (sec). */
    public static double measureWindowSec = 0.5;
    /** Pause between steps (sec). */
    public static double pauseBetweenStepsSec = 0.8;
    /** Power used to reposition the turret before each step. Must be well above
     *  CRServo stiction (~0.08-0.10) so proportional ramp still moves the turret. */
    public static double repositionPower = 0.15;
    /** Target position margin from the limit (deg) to reposition to before each step.
     *  Must be larger than softLimitMarginDeg (25) so the target is outside the
     *  attenuation zone; otherwise stiction + attenuation stalls repositioning. */
    public static double repositionMarginDeg = 30.0;
    /** Time (sec) the turret must remain near the start position with low velocity
     *  before the RUNNING phase begins. Prevents residual drift from corrupting data. */
    public static double settleTimeSec = 0.4;
    /** Maximum velocity (deg/s) considered "settled" after repositioning. */
    public static double settleVelThreshold = 5.0;
    /** Maximum position error (deg) from target to be considered "settled".
     *  Must be generous — the turret coasts past target during deceleration. */
    public static double settlePosThreshold = 10.0;
    /** Manual nudge power scale. */
    public static double nudgePowerScale = 0.15;
    /** Nudge deadband. */
    public static double nudgeDeadband = 0.05;

    //---------------- State ----------------
    private TurretHardware hardware;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private enum Phase { IDLE, REPOSITIONING, SETTLING, RUNNING, PAUSING, DONE }
    private Phase phase = Phase.IDLE;
    private boolean directionCW = true;
    private int currentStep = 0;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean settledOnce = false; // true once vel+pos thresholds met

    // Measurement accumulator
    private double velocitySum = 0.0;
    private int velocitySamples = 0;

    // Results
    private final double[] resultPower = new double[MAX_STEPS];
    private final double[] resultVelocity = new double[MAX_STEPS];
    private final boolean[] resultTruncated = new boolean[MAX_STEPS];
    private int resultCount = 0;
    private double computedKV = Double.NaN;
    private double computedKS = Double.NaN;
    private double repositionTargetDeg = 0.0; // cached for settle check

    @Override
    public void init() {
        hardware = new TurretHardware(hardwareMap);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
    }

    @Override
    public void start() {
        hardware.update();
        phase = Phase.IDLE;
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);
        hardware.update();

        double velDegSec = hardware.getVelocityDegPerSec();
        double posDeg = hardware.getPositionDeg();

        switch (phase) {
            case IDLE:
                handleIdle();
                break;
            case REPOSITIONING:
                handleRepositioning(posDeg);
                break;
            case SETTLING:
                handleSettling(posDeg, velDegSec);
                break;
            case RUNNING:
                handleRunning(velDegSec);
                break;
            case PAUSING:
                handlePausing();
                break;
            case DONE:
                hardware.setPower(0.0);
                break;
        }

        // Direction toggle (only when idle)
        if (phase == Phase.IDLE && edge(current.y, previous.y)) {
            directionCW = !directionCW;
        }

        // Start
        if (edge(current.a, previous.a) && (phase == Phase.IDLE || phase == Phase.DONE)) {
            startSequence();
        }

        // Abort
        if (edge(current.b, previous.b) && phase != Phase.IDLE) {
            hardware.setPower(0.0);
            phase = Phase.IDLE;
        }

        // --- Logging ---
        Logger.recordOutput(LOG_PREFIX + "Phase", phase);
        Logger.recordOutput(LOG_PREFIX + "DirectionCW", directionCW);
        Logger.recordOutput(LOG_PREFIX + "CurrentStep", currentStep);
        Logger.recordOutput(LOG_PREFIX + "TotalSteps", Math.min(powerSteps, MAX_STEPS));
        Logger.recordOutput(LOG_PREFIX + "PositionDeg", posDeg);
        Logger.recordOutput(LOG_PREFIX + "VelocityDegSec", velDegSec);
        Logger.recordOutput(LOG_PREFIX + "CommandedPower", hardware.getLastPower());
        Logger.recordOutput(LOG_PREFIX + "ResultCount", resultCount);

        // --- Telemetry ---
        joinedTelemetry.addData("Phase", phase);
        joinedTelemetry.addData("Direction", directionCW ? "CW (+)" : "CCW (-)");
        joinedTelemetry.addData("Step", currentStep + "/" + Math.min(powerSteps, MAX_STEPS));
        joinedTelemetry.addData("Position Deg", fmt(posDeg));
        joinedTelemetry.addData("Velocity Deg/s", fmt(velDegSec));
        joinedTelemetry.addData("Commanded Power", fmt(hardware.getLastPower()));

        if (phase == Phase.REPOSITIONING) {
            joinedTelemetry.addData("Reposition Target", fmt(repositionTargetDeg) + "°");
            joinedTelemetry.addData("Reposition Error", fmt(repositionTargetDeg - posDeg) + "°");
        } else if (phase == Phase.SETTLING) {
            joinedTelemetry.addData("Settle Target", fmt(repositionTargetDeg) + "°");
            joinedTelemetry.addData("Settle PosErr", fmt(Math.abs(repositionTargetDeg - posDeg)) + "° (< " + fmt(settlePosThreshold) + ")");
            joinedTelemetry.addData("Settle Vel", fmt(Math.abs(velDegSec)) + "°/s (< " + fmt(settleVelThreshold) + ")");
            joinedTelemetry.addData("Settled", settledOnce ? fmt(settleTimer.seconds()) + "/" + fmt(settleTimeSec) : "waiting...");
        } else if (phase == Phase.RUNNING) {
            double stepPower = getPowerForStep(currentStep);
            double elapsed = stepTimer.seconds();
            joinedTelemetry.addData("Current Step Power", fmt(stepPower));
            joinedTelemetry.addData("Step Elapsed", fmt(elapsed) + "/" + fmt(steadyStateDurationSec));
            if (elapsed >= (steadyStateDurationSec - measureWindowSec)) {
                joinedTelemetry.addData("Measuring Avg Vel", fmt(velocitySamples > 0 ? velocitySum / velocitySamples : 0));
            }
        }

        joinedTelemetry.addLine("--- Results (" + resultCount + ") ---");
        for (int i = 0; i < resultCount; i++) {
            String tag = resultTruncated[i] ? " [TRUNC]" : "";
            joinedTelemetry.addData("  Power=" + fmt(resultPower[i]),
                    "Vel=" + fmt(resultVelocity[i]) + " deg/s" + tag);
        }

        if (!Double.isNaN(computedKV)) {
            joinedTelemetry.addLine("=== COMPUTED GAINS ===");
            joinedTelemetry.addData("Suggested kV", fmt(computedKV));
            joinedTelemetry.addData("Suggested kS (intercept)", fmt(computedKS));
            joinedTelemetry.addLine("kV = power / velocity (steady state)");

            Logger.recordOutput(LOG_PREFIX + "ComputedKV", computedKV);
            Logger.recordOutput(LOG_PREFIX + "ComputedKS", computedKS);
        }

        joinedTelemetry.addLine("--- Controls ---");
        joinedTelemetry.addLine("A: start | B: abort | Y: toggle dir | LStick: nudge");
        joinedTelemetry.update();
    }

    private void startSequence() {
        resultCount = 0;
        currentStep = 0;
        computedKV = Double.NaN;
        computedKS = Double.NaN;
        stepTimer.reset();
        velocitySum = 0.0;
        velocitySamples = 0;
        settledOnce = false;
        phase = Phase.REPOSITIONING;
    }

    private void handleIdle() {
        double nudge = current.left_stick_x;
        if (Math.abs(nudge) < nudgeDeadband) nudge = 0.0;
        hardware.setPower(nudge * nudgePowerScale);
    }

    /**
     * Move the turret opposite to the test direction so there is room to run at speed.
     * If testing CW (positive), drive CCW toward the min limit + margin.
     * If testing CCW (negative), drive CW toward the max limit - margin.
     */
    private void handleRepositioning(double posDeg) {
        repositionTargetDeg = directionCW
                ? (TurretHardware.softLimitMinDeg + repositionMarginDeg)   // move toward low end
                : (TurretHardware.softLimitMaxDeg - repositionMarginDeg); // move toward high end
        double error = repositionTargetDeg - posDeg;
        double absDist = Math.abs(error);
        double vel = hardware.getVelocityDegPerSec();

        boolean posOk = absDist < settlePosThreshold;
        boolean velOk = Math.abs(vel) < settleVelThreshold;

        // Settled: position close AND velocity low for long enough → go to PAUSING
        if (posOk && velOk) {
            if (!settledOnce) {
                settledOnce = true;
                settleTimer.reset();
            }
            if (settleTimer.seconds() >= settleTimeSec) {
                hardware.setPower(0.0);
                stepTimer.reset();
                velocitySum = 0.0;
                velocitySamples = 0;
                phase = Phase.PAUSING;
                return;
            }
        } else {
            settledOnce = false;
        }

        // Active proportional control: drive toward target, ramp down within 10°.
        // When close and slow, apply zero power to let friction stop it.
        if (posOk && Math.abs(vel) < 30.0) {
            // Close enough and not moving fast — let friction brake
            hardware.setPower(0.0);
        } else {
            double scale = Math.min(1.0, absDist / 10.0);
            double power = (error > 0) ? repositionPower * scale : -repositionPower * scale;
            power = Math.max(-repositionPower, Math.min(repositionPower, power));
            // Min power to beat stiction, but only if not yet in settle zone
            if (!posOk && Math.abs(power) < 0.10) {
                power = Math.signum(power) * 0.10;
            }
            hardware.setPower(power);
        }

        Logger.recordOutput(LOG_PREFIX + "Settle/PosError", absDist);
        Logger.recordOutput(LOG_PREFIX + "Settle/PosOk", posOk);
        Logger.recordOutput(LOG_PREFIX + "Settle/VelOk", velOk);
    }

    /**
     * SETTLING phase is no longer used — settle logic is folded into REPOSITIONING
     * to maintain active position control and prevent coast oscillation.
     * Kept as a no-op in case the phase enum value appears in old code paths.
     */
    private void handleSettling(double posDeg, double velDegSec) {
        // Redirect to repositioning — should not normally be reached
        phase = Phase.REPOSITIONING;
    }

    private void handleRunning(double velDegSec) {
        int totalSteps = Math.min(powerSteps, MAX_STEPS);
        double stepPower = getPowerForStep(currentStep);
        double appliedPower = directionCW ? stepPower : -stepPower;

        // --- Soft-limit guard: auto-reverse if approaching limit ---
        if (hardware.isAtSoftLimit(directionCW) || hardware.isNearSoftLimit()) {
            // Check if we're in the measurement window and limit interference is active
            boolean inMeasureWindow = stepTimer.seconds() >= (steadyStateDurationSec - measureWindowSec);

            // Record partial measurement if we have enough clean samples
            if (velocitySamples > 3) {
                double avgVelocity = velocitySum / velocitySamples;
                resultPower[resultCount] = stepPower;
                resultVelocity[resultCount] = avgVelocity;
                resultTruncated[resultCount] = true;
                resultCount++;
                Logger.recordOutput(LOG_PREFIX + "Step/Power", stepPower);
                Logger.recordOutput(LOG_PREFIX + "Step/AvgVelocity", avgVelocity);
                Logger.recordOutput(LOG_PREFIX + "Step/Samples", velocitySamples);
                Logger.recordOutput(LOG_PREFIX + "Step/LimitTruncated", true);
            } else {
                // Not enough samples — skip this step entirely
                Logger.recordOutput(LOG_PREFIX + "Step/Power", stepPower);
                Logger.recordOutput(LOG_PREFIX + "Step/AvgVelocity", 0.0);
                Logger.recordOutput(LOG_PREFIX + "Step/Samples", velocitySamples);
                Logger.recordOutput(LOG_PREFIX + "Step/LimitTruncated", true);
                Logger.recordOutput(LOG_PREFIX + "Step/Skipped", true);
            }
            // Hit limit — advance to next step and reposition
            currentStep++;
            if (currentStep >= totalSteps) {
                hardware.setPower(0.0);
                computeKV();
                phase = Phase.DONE;
            } else {
                hardware.setPower(0.0);
                settledOnce = false;
                phase = Phase.REPOSITIONING;
            }
            return;
        }

        hardware.setPower(appliedPower);

        double elapsed = stepTimer.seconds();

        // Accumulate velocity during the measurement window, but ONLY if
        // the actual applied power matches the commanded power (no soft-limit
        // attenuation corrupting the measurement).
        if (elapsed >= (steadyStateDurationSec - measureWindowSec)) {
            velocitySum += Math.abs(velDegSec);
            velocitySamples++;
        }

        // Step complete
        if (elapsed >= steadyStateDurationSec) {
            double avgVelocity = (velocitySamples > 0) ? (velocitySum / velocitySamples) : 0.0;
            resultPower[resultCount] = stepPower;
            resultVelocity[resultCount] = avgVelocity;
            resultTruncated[resultCount] = false;
            resultCount++;

            Logger.recordOutput(LOG_PREFIX + "Step/Power", stepPower);
            Logger.recordOutput(LOG_PREFIX + "Step/AvgVelocity", avgVelocity);
            Logger.recordOutput(LOG_PREFIX + "Step/Samples", velocitySamples);
            Logger.recordOutput(LOG_PREFIX + "Step/LimitTruncated", false);

            currentStep++;
            if (currentStep >= totalSteps) {
                hardware.setPower(0.0);
                computeKV();
                phase = Phase.DONE;
            } else {
                // Reposition before next step
                hardware.setPower(0.0);
                settledOnce = false;
                phase = Phase.REPOSITIONING;
            }
        }
    }

    private void handlePausing() {
        hardware.setPower(0.0);
        if (stepTimer.seconds() >= pauseBetweenStepsSec) {
            stepTimer.reset();
            velocitySum = 0.0;
            velocitySamples = 0;
            phase = Phase.RUNNING;
        }
    }

    /**
     * Compute kV from the results using least-squares: power = kS + kV * velocity.
     * kV is the slope, kS is the intercept.
     * Only uses non-truncated (clean) measurements for the regression.
     */
    private void computeKV() {
        // Count clean (non-truncated) results
        int cleanCount = 0;
        for (int i = 0; i < resultCount; i++) {
            if (!resultTruncated[i]) cleanCount++;
        }

        if (cleanCount < 2) {
            // Fall back to all results if not enough clean ones
            cleanCount = resultCount;
            if (cleanCount < 2) return;
            // Use all results
            computeKVFromRange(0, resultCount, false);
        } else {
            computeKVFromRange(0, resultCount, true);
        }
    }

    /** Least-squares regression. If cleanOnly, skips truncated results. */
    private void computeKVFromRange(int start, int end, boolean cleanOnly) {
        double sumP = 0, sumV = 0;
        int n = 0;
        for (int i = start; i < end; i++) {
            if (cleanOnly && resultTruncated[i]) continue;
            sumP += resultPower[i];
            sumV += resultVelocity[i];
            n++;
        }
        if (n < 2) return;
        double meanP = sumP / n;
        double meanV = sumV / n;

        double num = 0, den = 0;
        for (int i = start; i < end; i++) {
            if (cleanOnly && resultTruncated[i]) continue;
            double dV = resultVelocity[i] - meanV;
            double dP = resultPower[i] - meanP;
            num += dV * dP;
            den += dV * dV;
        }

        if (Math.abs(den) < 1e-12) {
            computedKV = 0.0;
            computedKS = meanP;
        } else {
            computedKV = num / den;
            computedKS = meanP - computedKV * meanV;
        }

        Logger.recordOutput(LOG_PREFIX + "CleanStepsUsed", n);
    }

    private double getPowerForStep(int step) {
        int totalSteps = Math.min(powerSteps, MAX_STEPS);
        if (totalSteps <= 1) return startPower;
        return startPower + (endPower - startPower) * step / (totalSteps - 1);
    }

    private boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private String fmt(double v) {
        if (Double.isNaN(v)) return "---";
        return String.format("%.4f", v);
    }
}
