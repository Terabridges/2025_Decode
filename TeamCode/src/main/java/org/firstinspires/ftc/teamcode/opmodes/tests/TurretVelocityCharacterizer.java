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
    /** Starting power level (should be above kS). */
    public static double startPower = 0.10;
    /** Ending power level. */
    public static double endPower = 0.80;
    /** Number of power steps to test. */
    public static int powerSteps = 8;
    /** Duration at each power level to reach steady state (sec). */
    public static double steadyStateDurationSec = 1.5;
    /** Window at end of each step to measure average velocity (sec). */
    public static double measureWindowSec = 0.5;
    /** Pause between steps (sec). */
    public static double pauseBetweenStepsSec = 0.8;
    /** Manual nudge power scale. */
    public static double nudgePowerScale = 0.15;
    /** Nudge deadband. */
    public static double nudgeDeadband = 0.05;

    //---------------- State ----------------
    private TurretHardware hardware;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private enum Phase { IDLE, RUNNING, PAUSING, DONE }
    private Phase phase = Phase.IDLE;
    private boolean directionCW = true;
    private int currentStep = 0;
    private final ElapsedTime stepTimer = new ElapsedTime();

    // Measurement accumulator
    private double velocitySum = 0.0;
    private int velocitySamples = 0;

    // Results
    private final double[] resultPower = new double[MAX_STEPS];
    private final double[] resultVelocity = new double[MAX_STEPS];
    private int resultCount = 0;
    private double computedKV = Double.NaN;
    private double computedKS = Double.NaN;

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

        if (phase == Phase.RUNNING) {
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
            joinedTelemetry.addData("  Power=" + fmt(resultPower[i]),
                    "Vel=" + fmt(resultVelocity[i]) + " deg/s");
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
        phase = Phase.RUNNING;
    }

    private void handleIdle() {
        double nudge = current.left_stick_x;
        if (Math.abs(nudge) < nudgeDeadband) nudge = 0.0;
        hardware.setPower(nudge * nudgePowerScale);
    }

    private void handleRunning(double velDegSec) {
        int totalSteps = Math.min(powerSteps, MAX_STEPS);
        double stepPower = getPowerForStep(currentStep);
        double appliedPower = directionCW ? stepPower : -stepPower;
        hardware.setPower(appliedPower);

        double elapsed = stepTimer.seconds();

        // Accumulate velocity during the measurement window
        if (elapsed >= (steadyStateDurationSec - measureWindowSec)) {
            velocitySum += Math.abs(velDegSec);
            velocitySamples++;
        }

        // Step complete
        if (elapsed >= steadyStateDurationSec) {
            double avgVelocity = (velocitySamples > 0) ? (velocitySum / velocitySamples) : 0.0;
            resultPower[resultCount] = stepPower;
            resultVelocity[resultCount] = avgVelocity;
            resultCount++;

            Logger.recordOutput(LOG_PREFIX + "Step/Power", stepPower);
            Logger.recordOutput(LOG_PREFIX + "Step/AvgVelocity", avgVelocity);
            Logger.recordOutput(LOG_PREFIX + "Step/Samples", velocitySamples);

            currentStep++;
            if (currentStep >= totalSteps) {
                hardware.setPower(0.0);
                computeKV();
                phase = Phase.DONE;
            } else {
                // Pause between steps
                hardware.setPower(0.0);
                stepTimer.reset();
                velocitySum = 0.0;
                velocitySamples = 0;
                phase = Phase.PAUSING;
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
     */
    private void computeKV() {
        if (resultCount < 2) return;

        double sumP = 0, sumV = 0;
        for (int i = 0; i < resultCount; i++) {
            sumP += resultPower[i];
            sumV += resultVelocity[i];
        }
        double meanP = sumP / resultCount;
        double meanV = sumV / resultCount;

        double num = 0, den = 0;
        for (int i = 0; i < resultCount; i++) {
            double dV = resultVelocity[i] - meanV;
            double dP = resultPower[i] - meanP;
            num += dV * dP;
            den += dV * dV;
        }

        if (Math.abs(den) < 1e-12) {
            computedKV = 0.0;
            computedKS = meanP;
        } else {
            computedKV = num / den; // power per deg/sec
            computedKS = meanP - computedKV * meanV; // intercept = stiction estimate
        }
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
