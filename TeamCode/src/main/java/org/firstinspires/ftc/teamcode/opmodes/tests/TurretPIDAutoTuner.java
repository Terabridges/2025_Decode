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
 * Semi-automated PID auto-tuner for the CRServo turret using the relay feedback
 * method (Åström–Hägglund).
 *
 * <h3>Purpose</h3>
 * Determines PID gains ({@code kP}, {@code kI}, {@code kD}) by inducing sustained
 * oscillation via bang-bang control around a target position, then measuring the
 * ultimate period and amplitude to compute gains.
 *
 * <h3>How It Works</h3>
 * <ol>
 *   <li>Set the turret at a safe position (manually or via nudge).</li>
 *   <li>Press A to start relay oscillation around the current position.</li>
 *   <li>The controller applies +{@code relayPower} when below target and
 *       -{@code relayPower} when above target (with hysteresis band).</li>
 *   <li>After {@code requiredOscillations} zero-crossings, the ultimate period
 *       (T_u) and amplitude (A) are computed.</li>
 *   <li>PID gains are computed using the selected tuning rule (Ziegler-Nichols
 *       or Tyreus-Luyben).</li>
 *   <li>Press B to abort at any time.</li>
 * </ol>
 *
 * <h3>Controls</h3>
 * <ul>
 *   <li><b>A</b> — start/restart relay oscillation</li>
 *   <li><b>B</b> — abort and stop</li>
 *   <li><b>Y</b> — toggle tuning rule (ZN ↔ TL)</li>
 *   <li><b>X</b> — set target to current position</li>
 *   <li><b>Dpad Up/Down</b> — adjust relay power ±0.01</li>
 *   <li><b>Left Stick X</b> — manual nudge when idle</li>
 * </ul>
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretPIDAutoTuner", group = "Test")
public class TurretPIDAutoTuner extends OpMode {
    private static final String LOG_PREFIX = "Calibrator/PIDAutoTune/";
    private static final int MAX_CROSSINGS = 40;

    //---------------- Configurable ----------------
    /** Power for relay (bang-bang) control. Should be below max safe but enough to move. */
    public static double relayPower = 0.20;
    /** Hysteresis band around target (degrees). Prevents chattering at the crossing. */
    public static double hysteresisDeg = 1.0;
    /** Number of full oscillation cycles needed for reliable measurement. */
    public static int requiredOscillations = 6;
    /** Maximum time to wait for oscillations before aborting (sec). */
    public static double maxTestDurationSec = 30.0;
    /** Manual nudge power scale. */
    public static double nudgePowerScale = 0.15;
    /** Nudge deadband. */
    public static double nudgeDeadband = 0.05;
    /** Target position for relay oscillation. */
    public static double relayTargetDeg = 180.0;

    //---------------- State ----------------
    private TurretHardware hardware;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private enum Phase { IDLE, OSCILLATING, DONE }
    private Phase phase = Phase.IDLE;
    private boolean useZieglerNichols = true; // false = Tyreus-Luyben
    private final ElapsedTime testTimer = new ElapsedTime();

    // Oscillation tracking
    private boolean lastAboveTarget = false;
    private final double[] crossingTimeSec = new double[MAX_CROSSINGS];
    private int crossingCount = 0;
    private double peakMax = Double.NEGATIVE_INFINITY;
    private double peakMin = Double.POSITIVE_INFINITY;

    // Results
    private double ultimatePeriod = Double.NaN;
    private double ultimateAmplitude = Double.NaN;
    private double ultimateGainKu = Double.NaN;
    private double suggestedKP = Double.NaN;
    private double suggestedKI = Double.NaN;
    private double suggestedKD = Double.NaN;

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
        relayTargetDeg = hardware.getPositionDeg();
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
            case OSCILLATING:
                handleOscillating(posDeg);
                break;
            case DONE:
                hardware.setPower(0.0);
                break;
        }

        // --- Start ---
        if (edge(current.a, previous.a) && (phase == Phase.IDLE || phase == Phase.DONE)) {
            startOscillation(posDeg);
        }

        // --- Abort ---
        if (edge(current.b, previous.b) && phase != Phase.IDLE) {
            hardware.setPower(0.0);
            phase = Phase.IDLE;
        }

        // --- Toggle tuning rule ---
        if (edge(current.y, previous.y)) {
            useZieglerNichols = !useZieglerNichols;
            if (!Double.isNaN(ultimatePeriod)) {
                computePIDGains(); // recompute with new rule
            }
        }

        // --- Set target to current ---
        if (edge(current.x, previous.x) && phase == Phase.IDLE) {
            relayTargetDeg = posDeg;
        }

        // --- Adjust relay power ---
        if (edge(current.dpad_up, previous.dpad_up)) relayPower += 0.01;
        if (edge(current.dpad_down, previous.dpad_down)) relayPower = Math.max(0.02, relayPower - 0.01);

        // --- Logging ---
        Logger.recordOutput(LOG_PREFIX + "Phase", phase);
        Logger.recordOutput(LOG_PREFIX + "RelayTargetDeg", relayTargetDeg);
        Logger.recordOutput(LOG_PREFIX + "RelayPower", relayPower);
        Logger.recordOutput(LOG_PREFIX + "PositionDeg", posDeg);
        Logger.recordOutput(LOG_PREFIX + "VelocityDegSec", velDegSec);
        Logger.recordOutput(LOG_PREFIX + "CrossingCount", crossingCount);
        Logger.recordOutput(LOG_PREFIX + "TuningRule", useZieglerNichols ? "Ziegler-Nichols" : "Tyreus-Luyben");

        if (!Double.isNaN(ultimatePeriod)) {
            Logger.recordOutput(LOG_PREFIX + "UltimatePeriod", ultimatePeriod);
            Logger.recordOutput(LOG_PREFIX + "UltimateAmplitude", ultimateAmplitude);
            Logger.recordOutput(LOG_PREFIX + "UltimateGainKu", ultimateGainKu);
            Logger.recordOutput(LOG_PREFIX + "SuggestedKP", suggestedKP);
            Logger.recordOutput(LOG_PREFIX + "SuggestedKI", suggestedKI);
            Logger.recordOutput(LOG_PREFIX + "SuggestedKD", suggestedKD);
        }

        // --- Telemetry ---
        joinedTelemetry.addData("Phase", phase);
        joinedTelemetry.addData("Relay Target Deg", fmt(relayTargetDeg));
        joinedTelemetry.addData("Relay Power", fmt(relayPower));
        joinedTelemetry.addData("Position Deg", fmt(posDeg));
        joinedTelemetry.addData("Velocity Deg/s", fmt(velDegSec));
        joinedTelemetry.addData("Tuning Rule", useZieglerNichols ? "Ziegler-Nichols" : "Tyreus-Luyben");

        if (phase == Phase.OSCILLATING) {
            int required = requiredOscillations * 2; // crossings per full oscillation
            joinedTelemetry.addData("Crossings", crossingCount + "/" + required);
            joinedTelemetry.addData("Elapsed", fmt(testTimer.seconds()) + "s");
            joinedTelemetry.addData("Peak Max", fmt(peakMax));
            joinedTelemetry.addData("Peak Min", fmt(peakMin));
        }

        if (!Double.isNaN(ultimatePeriod)) {
            joinedTelemetry.addLine("=== RESULTS ===");
            joinedTelemetry.addData("Ultimate Period (Tu)", fmt(ultimatePeriod) + " sec");
            joinedTelemetry.addData("Ultimate Amplitude (A)", fmt(ultimateAmplitude) + " deg");
            joinedTelemetry.addData("Ultimate Gain (Ku)", fmt(ultimateGainKu));
            joinedTelemetry.addLine("--- Suggested PID ---");
            joinedTelemetry.addData("kP", fmt(suggestedKP));
            joinedTelemetry.addData("kI", fmt(suggestedKI));
            joinedTelemetry.addData("kD", fmt(suggestedKD));
            joinedTelemetry.addLine("Apply to TurretController.java");
        }

        joinedTelemetry.addLine("--- Controls ---");
        joinedTelemetry.addLine("A: start | B: abort | Y: toggle rule | X: set target");
        joinedTelemetry.addLine("Dpad UD: relay power ±0.01 | LStick X: nudge");
        joinedTelemetry.update();
    }

    private void handleIdle() {
        double nudge = current.left_stick_x;
        if (Math.abs(nudge) < nudgeDeadband) nudge = 0.0;
        hardware.setPower(nudge * nudgePowerScale);
    }

    private void startOscillation(double currentDeg) {
        // Clamp relay target to safe range
        relayTargetDeg = TurretHardware.clampToSafeRange(relayTargetDeg);
        // Also ensure target has enough room for oscillation on both sides
        double minSafe = TurretHardware.softLimitMinDeg + 20.0;
        double maxSafe = TurretHardware.softLimitMaxDeg - 20.0;
        relayTargetDeg = Math.max(minSafe, Math.min(maxSafe, relayTargetDeg));

        crossingCount = 0;
        peakMax = Double.NEGATIVE_INFINITY;
        peakMin = Double.POSITIVE_INFINITY;
        ultimatePeriod = Double.NaN;
        ultimateAmplitude = Double.NaN;
        ultimateGainKu = Double.NaN;
        suggestedKP = Double.NaN;
        suggestedKI = Double.NaN;
        suggestedKD = Double.NaN;
        lastAboveTarget = (currentDeg >= relayTargetDeg);
        testTimer.reset();
        phase = Phase.OSCILLATING;
    }

    private void handleOscillating(double posDeg) {
        // Safety: abort if we've drifted near a limit
        if (hardware.isNearSoftLimit()) {
            hardware.setPower(0.0);
            if (crossingCount >= 4) {
                computeResults();
            }
            phase = Phase.DONE;
            Logger.recordOutput(LOG_PREFIX + "AbortedAtLimit", true);
            return;
        }

        double error = wrapSigned(posDeg - relayTargetDeg);
        boolean aboveTarget = (error > hysteresisDeg);
        boolean belowTarget = (error < -hysteresisDeg);

        // Track peaks
        if (posDeg > peakMax) peakMax = posDeg;
        if (posDeg < peakMin) peakMin = posDeg;

        // Relay output with hysteresis
        if (aboveTarget) {
            hardware.setPower(-relayPower);
            if (!lastAboveTarget) {
                // Crossed from below to above
                recordCrossing();
                lastAboveTarget = true;
            }
        } else if (belowTarget) {
            hardware.setPower(relayPower);
            if (lastAboveTarget) {
                // Crossed from above to below
                recordCrossing();
                lastAboveTarget = false;
            }
        }
        // else: in hysteresis band, keep previous command

        // Safety timeout
        if (testTimer.seconds() >= maxTestDurationSec) {
            hardware.setPower(0.0);
            if (crossingCount >= 4) {
                computeResults();
            }
            phase = Phase.DONE;
        }
    }

    private void recordCrossing() {
        if (crossingCount < MAX_CROSSINGS) {
            crossingTimeSec[crossingCount] = testTimer.seconds();
            crossingCount++;

            Logger.recordOutput(LOG_PREFIX + "Crossing/Time", testTimer.seconds());
            Logger.recordOutput(LOG_PREFIX + "Crossing/Count", crossingCount);

            // Reset peak tracking after each half-cycle
            if (crossingCount > 1) {
                // Keep running peaks for amplitude, but log them
                Logger.recordOutput(LOG_PREFIX + "Crossing/PeakMax", peakMax);
                Logger.recordOutput(LOG_PREFIX + "Crossing/PeakMin", peakMin);
            }
        }

        // Check if we have enough oscillations
        int requiredCrossings = requiredOscillations * 2;
        if (crossingCount >= requiredCrossings) {
            hardware.setPower(0.0);
            computeResults();
            phase = Phase.DONE;
        }
    }

    private void computeResults() {
        if (crossingCount < 4) return;

        // Skip the first 2 crossings (transient), compute period from remaining
        int startIdx = 2;
        int usableCrossings = crossingCount - startIdx;
        if (usableCrossings < 2) {
            startIdx = 0;
            usableCrossings = crossingCount;
        }

        // Average half-period from consecutive crossings
        double totalTime = crossingTimeSec[crossingCount - 1] - crossingTimeSec[startIdx];
        int halfPeriods = usableCrossings - 1;
        double avgHalfPeriod = totalTime / halfPeriods;
        ultimatePeriod = avgHalfPeriod * 2.0; // full period

        // Amplitude from peak-to-peak
        ultimateAmplitude = (peakMax - peakMin) / 2.0;

        // Ultimate gain: Ku = 4d / (π * A) where d = relay amplitude
        if (ultimateAmplitude > 0.01) {
            ultimateGainKu = (4.0 * relayPower) / (Math.PI * ultimateAmplitude);
        } else {
            ultimateGainKu = 0.0;
        }

        computePIDGains();
    }

    private void computePIDGains() {
        if (Double.isNaN(ultimatePeriod) || Double.isNaN(ultimateGainKu)) return;

        double ku = ultimateGainKu;
        double tu = ultimatePeriod;

        if (useZieglerNichols) {
            // Ziegler-Nichols PID
            suggestedKP = 0.6 * ku;
            suggestedKI = 2.0 * suggestedKP / tu;    // Ki = Kp / (Tu/2)
            suggestedKD = suggestedKP * tu / 8.0;     // Kd = Kp * Tu/8
        } else {
            // Tyreus-Luyben (less aggressive, better for position control)
            suggestedKP = 0.45 * ku;
            suggestedKI = suggestedKP / (2.2 * tu);   // Ki = Kp / (2.2*Tu)
            suggestedKD = suggestedKP * tu / 6.3;     // Kd = Kp * Tu/6.3
        }
    }

    private double wrapSigned(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private String fmt(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "---";
        return String.format("%.4f", v);
    }
}
