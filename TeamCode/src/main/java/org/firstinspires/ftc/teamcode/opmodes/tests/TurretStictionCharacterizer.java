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
 * Automated stiction (static friction) characterizer for the CRServo turret.
 *
 * <h3>Purpose</h3>
 * Determines the minimum CRServo power needed to overcome static friction and
 * start moving the turret. This is the {@code kS} feedforward gain used by
 * {@link org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretController}.
 *
 * <h3>How It Works</h3>
 * <ol>
 *   <li>Moves to a test position (via manual nudge or dashboard).</li>
 *   <li>Press A to start a ramp test: power increases from 0 at {@code rampRatePerSec}
 *       per second until the turret velocity exceeds {@code breakawayThresholdDegSec}.</li>
 *   <li>Records the breakaway power for the current direction (CW or CCW).</li>
 *   <li>Press B to toggle direction (CW ↔ CCW).</li>
 *   <li>Press Y to capture the pair at the current position.</li>
 *   <li>Repeat at multiple positions to map stiction across the turret range.</li>
 *   <li>Press X to clear all captures.</li>
 * </ol>
 *
 * <h3>Controls</h3>
 * <ul>
 *   <li><b>Left Stick X</b> — manual nudge (when not ramping)</li>
 *   <li><b>A</b> — start ramp test</li>
 *   <li><b>B</b> — toggle CW/CCW direction</li>
 *   <li><b>Y</b> — capture breakaway pair at current position</li>
 *   <li><b>X</b> — clear all captures</li>
 *   <li><b>BACK</b> — abort ramp (safety stop)</li>
 * </ul>
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretStictionCharacterizer", group = "Test")
public class TurretStictionCharacterizer extends OpMode {
    private static final String LOG_PREFIX = "Calibrator/Stiction/";
    private static final int MAX_CAPTURES = 16;

    //---------------- Configurable ----------------
    /** Power ramp rate (power units per second). */
    public static double rampRatePerSec = 0.05;
    /** Velocity threshold to detect breakaway (deg/sec). */
    public static double breakawayThresholdDegSec = 5.0;
    /** Settle time before starting ramp (seconds). */
    public static double settleTimeSec = 0.5;
    /** Maximum power before aborting ramp (safety). */
    public static double maxRampPower = 0.5;
    /** Manual nudge power scale. */
    public static double nudgePowerScale = 0.15;
    /** Nudge deadband. */
    public static double nudgeDeadband = 0.05;

    //---------------- State ----------------
    private TurretHardware hardware;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private enum Phase { IDLE, SETTLING, RAMPING, DONE }
    private Phase phase = Phase.IDLE;
    private boolean directionCW = true; // true = positive power
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private double currentRampPower = 0.0;
    private double lastBreakawayPowerCW = Double.NaN;
    private double lastBreakawayPowerCCW = Double.NaN;

    // Captures: per-position breakaway pairs
    private final double[] capturePositionDeg = new double[MAX_CAPTURES];
    private final double[] captureKsCW = new double[MAX_CAPTURES];
    private final double[] captureKsCCW = new double[MAX_CAPTURES];
    private int captureCount = 0;

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
        double posDeg = hardware.getPositionDeg();
        double velDegSec = hardware.getVelocityDegPerSec();

        switch (phase) {
            case IDLE:
                handleIdle();
                break;
            case SETTLING:
                handleSettling();
                break;
            case RAMPING:
                handleRamping(velDegSec);
                break;
            case DONE:
                hardware.setPower(0.0);
                if (edge(current.a, previous.a)) {
                    phase = Phase.IDLE;
                }
                break;
        }

        // --- Direction toggle (only when idle) ---
        if (phase == Phase.IDLE && edge(current.b, previous.b)) {
            directionCW = !directionCW;
        }

        // --- Capture breakaway pair ---
        if (edge(current.y, previous.y)
                && !Double.isNaN(lastBreakawayPowerCW)
                && !Double.isNaN(lastBreakawayPowerCCW)
                && captureCount < MAX_CAPTURES) {
            capturePositionDeg[captureCount] = posDeg;
            captureKsCW[captureCount] = lastBreakawayPowerCW;
            captureKsCCW[captureCount] = lastBreakawayPowerCCW;
            captureCount++;
        }

        // --- Clear ---
        if (edge(current.x, previous.x)) {
            captureCount = 0;
            lastBreakawayPowerCW = Double.NaN;
            lastBreakawayPowerCCW = Double.NaN;
        }

        // --- Abort ---
        if (edge(current.back, previous.back) && phase != Phase.IDLE) {
            hardware.setPower(0.0);
            phase = Phase.IDLE;
        }

        // --- Logging ---
        Logger.recordOutput(LOG_PREFIX + "Phase", phase);
        Logger.recordOutput(LOG_PREFIX + "DirectionCW", directionCW);
        Logger.recordOutput(LOG_PREFIX + "RampPower", currentRampPower);
        Logger.recordOutput(LOG_PREFIX + "PositionDeg", posDeg);
        Logger.recordOutput(LOG_PREFIX + "VelocityDegSec", velDegSec);
        Logger.recordOutput(LOG_PREFIX + "BreakawayPowerCW", orNaN(lastBreakawayPowerCW));
        Logger.recordOutput(LOG_PREFIX + "BreakawayPowerCCW", orNaN(lastBreakawayPowerCCW));
        Logger.recordOutput(LOG_PREFIX + "CaptureCount", captureCount);

        // --- Telemetry ---
        joinedTelemetry.addData("Phase", phase);
        joinedTelemetry.addData("Direction", directionCW ? "CW (+)" : "CCW (-)");
        joinedTelemetry.addData("Position Deg", fmt(posDeg));
        joinedTelemetry.addData("Velocity Deg/s", fmt(velDegSec));
        joinedTelemetry.addData("Ramp Power", fmt(currentRampPower));
        joinedTelemetry.addData("Last Breakaway CW", fmt(lastBreakawayPowerCW));
        joinedTelemetry.addData("Last Breakaway CCW", fmt(lastBreakawayPowerCCW));

        if (!Double.isNaN(lastBreakawayPowerCW) && !Double.isNaN(lastBreakawayPowerCCW)) {
            double avgKs = (lastBreakawayPowerCW + lastBreakawayPowerCCW) / 2.0;
            joinedTelemetry.addData("Suggested kS (avg)", fmt(avgKs));
            Logger.recordOutput(LOG_PREFIX + "SuggestedKs", avgKs);
        }

        joinedTelemetry.addLine("--- Captures (" + captureCount + ") ---");
        for (int i = 0; i < captureCount; i++) {
            joinedTelemetry.addData("  [" + i + "]",
                    fmt(capturePositionDeg[i]) + "° → CW=" + fmt(captureKsCW[i])
                            + " CCW=" + fmt(captureKsCCW[i])
                            + " avg=" + fmt((captureKsCW[i] + captureKsCCW[i]) / 2.0));
        }

        if (captureCount > 0) {
            double sumKs = 0;
            for (int i = 0; i < captureCount; i++) {
                sumKs += (captureKsCW[i] + captureKsCCW[i]) / 2.0;
            }
            double globalAvg = sumKs / captureCount;
            joinedTelemetry.addData("Global Avg kS", fmt(globalAvg));
            Logger.recordOutput(LOG_PREFIX + "GlobalAvgKs", globalAvg);
        }

        joinedTelemetry.addLine("--- Controls ---");
        joinedTelemetry.addLine("A: start ramp | B: toggle dir | Y: capture pair");
        joinedTelemetry.addLine("X: clear | BACK: abort | LStick X: nudge");
        joinedTelemetry.update();
    }

    private void handleIdle() {
        // Manual nudge
        double nudge = current.left_stick_x;
        if (Math.abs(nudge) < nudgeDeadband) nudge = 0.0;
        hardware.setPower(nudge * nudgePowerScale);

        // Start ramp
        if (edge(current.a, previous.a)) {
            hardware.setPower(0.0);
            currentRampPower = 0.0;
            phaseTimer.reset();
            phase = Phase.SETTLING;
        }
    }

    private void handleSettling() {
        hardware.setPower(0.0);
        if (phaseTimer.seconds() >= settleTimeSec) {
            phaseTimer.reset();
            currentRampPower = 0.0;
            phase = Phase.RAMPING;
        }
    }

    private void handleRamping(double velDegSec) {
        double elapsed = phaseTimer.seconds();
        currentRampPower = rampRatePerSec * elapsed;

        // Apply direction
        double appliedPower = directionCW ? currentRampPower : -currentRampPower;
        hardware.setPower(appliedPower);

        // Soft-limit guard: abort ramp if we hit a limit
        if (hardware.isAtSoftLimit(directionCW)) {
            hardware.setPower(0.0);
            phase = Phase.DONE;
            Logger.recordOutput(LOG_PREFIX + "RampAbortedAtLimit", true);
            return;
        }

        // Check for breakaway
        if (Math.abs(velDegSec) >= breakawayThresholdDegSec) {
            if (directionCW) {
                lastBreakawayPowerCW = currentRampPower;
            } else {
                lastBreakawayPowerCCW = currentRampPower;
            }
            hardware.setPower(0.0);
            phase = Phase.DONE;

            Logger.recordOutput(LOG_PREFIX + "BreakawayDetected", true);
            Logger.recordOutput(LOG_PREFIX + "BreakawayPower", currentRampPower);
            Logger.recordOutput(LOG_PREFIX + "BreakawayDirection", directionCW ? "CW" : "CCW");
        }

        // Safety abort
        if (currentRampPower >= maxRampPower) {
            hardware.setPower(0.0);
            phase = Phase.DONE;
            Logger.recordOutput(LOG_PREFIX + "RampAborted", true);
        }
    }

    private double orNaN(double v) {
        return Double.isNaN(v) ? 0.0 : v;
    }

    private boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private String fmt(double v) {
        if (Double.isNaN(v)) return "---";
        return String.format("%.4f", v);
    }
}
