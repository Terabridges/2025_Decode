package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretHardware;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

/**
 * Multi-point encoder calibration OpMode for the CRServo turret.
 *
 * <h3>Workflow</h3>
 * <ol>
 *   <li>Manually position the turret to a known physical angle.</li>
 *   <li>Enter the true turret angle via the dashboard ({@code trueAngleDeg}).</li>
 *   <li>Press A to capture the (encoder, true) pair.</li>
 *   <li>Repeat for 4–8 positions spanning the turret range.</li>
 *   <li>Press Y to compute best-fit linear mapping and display results.</li>
 *   <li>Press X to clear all captures and start over.</li>
 * </ol>
 *
 * <h3>Controls</h3>
 * <ul>
 *   <li><b>Left Stick X</b> — nudge turret CRServo (low power) for fine positioning</li>
 *   <li><b>Dpad Left/Right</b> — step {@code trueAngleDeg} by ±{@code smallStepDeg}</li>
 *   <li><b>Dpad Up/Down</b> — step {@code trueAngleDeg} by ±{@code largeStepDeg}</li>
 *   <li><b>A</b> — capture current (encoder, true) pair</li>
 *   <li><b>Y</b> — compute best-fit calibration</li>
 *   <li><b>X</b> — clear all captures</li>
 *   <li><b>B</b> — stop servo (safety)</li>
 * </ul>
 *
 * After pressing Y, the telemetry displays suggested values for
 * {@code encoderRefDeg}, {@code encoderRefTurretDeg}, and {@code encoderToTurretScale}.
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretEncoderCalibrator", group = "Test")
public class TurretEncoderCalibrator extends OpMode {
    private static final String LOG_PREFIX = "Calibrator/Encoder/";
    private static final int MAX_CAPTURES = 16;

    //---------------- Configurable ----------------
    /** The true physical turret angle at the current position. Set via dashboard. */
    public static double trueAngleDeg = 180.0;
    /** Small angle step for dpad left/right. */
    public static double smallStepDeg = 1.0;
    /** Large angle step for dpad up/down. */
    public static double largeStepDeg = 10.0;
    /** Maximum nudge power from left stick X. */
    public static double nudgePowerScale = 0.15;
    /** Deadband for nudge joystick. */
    public static double nudgeDeadband = 0.05;

    //---------------- State ----------------
    private TurretHardware hardware;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    // Capture arrays
    private final double[] capturedEncoderDeg = new double[MAX_CAPTURES];
    private final double[] capturedTrueDeg = new double[MAX_CAPTURES];
    private int captureCount = 0;

    // Best-fit results
    private boolean fitComputed = false;
    private double fitRefEncoderDeg = Double.NaN;
    private double fitRefTurretDeg = Double.NaN;
    private double fitScale = Double.NaN;
    private double fitRSquared = Double.NaN;

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
        trueAngleDeg = hardware.getPositionDeg(); // seed with current mapped position
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);

        hardware.update();

        // --- Nudge servo ---
        double nudge = current.left_stick_x;
        if (Math.abs(nudge) < nudgeDeadband) nudge = 0.0;
        hardware.setPower(nudge * nudgePowerScale);

        // --- True angle adjustment ---
        if (edge(current.dpad_right, previous.dpad_right)) trueAngleDeg += smallStepDeg;
        if (edge(current.dpad_left, previous.dpad_left))  trueAngleDeg -= smallStepDeg;
        if (edge(current.dpad_up, previous.dpad_up))      trueAngleDeg += largeStepDeg;
        if (edge(current.dpad_down, previous.dpad_down))  trueAngleDeg -= largeStepDeg;

        // --- Safety stop ---
        if (current.b) {
            hardware.setPower(0.0);
        }

        // --- Capture ---
        if (edge(current.a, previous.a) && captureCount < MAX_CAPTURES) {
            capturedEncoderDeg[captureCount] = hardware.getEncoderRawDeg();
            capturedTrueDeg[captureCount] = trueAngleDeg;
            captureCount++;
            fitComputed = false;

            Logger.recordOutput(LOG_PREFIX + "LastCaptureEncoder", hardware.getEncoderRawDeg());
            Logger.recordOutput(LOG_PREFIX + "LastCaptureTrue", trueAngleDeg);
            Logger.recordOutput(LOG_PREFIX + "CaptureCount", captureCount);
        }

        // --- Compute fit ---
        if (edge(current.y, previous.y) && captureCount >= 2) {
            computeBestFit();
        }

        // --- Clear ---
        if (edge(current.x, previous.x)) {
            captureCount = 0;
            fitComputed = false;
        }

        // --- Logging ---
        double rawEnc = hardware.getEncoderRawDeg();
        double mappedPos = hardware.getPositionDeg();
        Logger.recordOutput(LOG_PREFIX + "RawEncoderDeg", rawEnc);
        Logger.recordOutput(LOG_PREFIX + "MappedPositionDeg", mappedPos);
        Logger.recordOutput(LOG_PREFIX + "TrueAngleDeg", trueAngleDeg);
        Logger.recordOutput(LOG_PREFIX + "NudgePower", nudge * nudgePowerScale);
        Logger.recordOutput(LOG_PREFIX + "CaptureCount", captureCount);
        Logger.recordOutput(LOG_PREFIX + "FitComputed", fitComputed);

        // --- Telemetry ---
        joinedTelemetry.addData("Raw Encoder Deg", fmt(rawEnc));
        joinedTelemetry.addData("Mapped Position Deg", fmt(mappedPos));
        joinedTelemetry.addData("True Angle (set)", fmt(trueAngleDeg));
        joinedTelemetry.addData("Encoder Voltage", fmt(hardware.getEncoderVoltage()));
        joinedTelemetry.addData("Nudge Power", fmt(nudge * nudgePowerScale));
        joinedTelemetry.addLine("--- Captures (" + captureCount + "/" + MAX_CAPTURES + ") ---");

        for (int i = 0; i < captureCount; i++) {
            joinedTelemetry.addData("  [" + i + "]",
                    "enc=" + fmt(capturedEncoderDeg[i]) + " → true=" + fmt(capturedTrueDeg[i]));
        }

        if (fitComputed) {
            joinedTelemetry.addLine("=== BEST-FIT RESULTS ===");
            joinedTelemetry.addData("Suggested encoderRefDeg", fmt(fitRefEncoderDeg));
            joinedTelemetry.addData("Suggested encoderRefTurretDeg", fmt(fitRefTurretDeg));
            joinedTelemetry.addData("Suggested encoderToTurretScale", fmt(fitScale));
            joinedTelemetry.addData("R²", fmt(fitRSquared));
            joinedTelemetry.addLine("Copy these into TurretHardware.java");

            Logger.recordOutput(LOG_PREFIX + "Fit/RefEncoderDeg", fitRefEncoderDeg);
            Logger.recordOutput(LOG_PREFIX + "Fit/RefTurretDeg", fitRefTurretDeg);
            Logger.recordOutput(LOG_PREFIX + "Fit/Scale", fitScale);
            Logger.recordOutput(LOG_PREFIX + "Fit/RSquared", fitRSquared);
        } else if (captureCount < 2) {
            joinedTelemetry.addLine("Need at least 2 captures to compute fit.");
        } else {
            joinedTelemetry.addLine("Press Y to compute best-fit.");
        }

        joinedTelemetry.addLine("--- Controls ---");
        joinedTelemetry.addLine("LStick X: nudge | Dpad LR: ±small | Dpad UD: ±large");
        joinedTelemetry.addLine("A: capture | Y: compute fit | X: clear | B: stop");
        joinedTelemetry.update();
    }

    /**
     * Compute least-squares linear fit: trueDeg = scale * (encoderDeg - refEncoder) + refTurret.
     * Uses the centroid as the reference point and computes R² for quality assessment.
     */
    private void computeBestFit() {
        if (captureCount < 2) return;

        // Compute means
        double sumEnc = 0, sumTrue = 0;
        for (int i = 0; i < captureCount; i++) {
            sumEnc += capturedEncoderDeg[i];
            sumTrue += capturedTrueDeg[i];
        }
        double meanEnc = sumEnc / captureCount;
        double meanTrue = sumTrue / captureCount;

        // Least-squares slope
        double num = 0, den = 0;
        for (int i = 0; i < captureCount; i++) {
            double dEnc = capturedEncoderDeg[i] - meanEnc;
            double dTrue = capturedTrueDeg[i] - meanTrue;
            num += dEnc * dTrue;
            den += dEnc * dEnc;
        }

        if (Math.abs(den) < 1e-9) {
            fitScale = 1.0; // all encoder readings the same — can't compute
        } else {
            fitScale = num / den;
        }

        // Reference point = centroid
        fitRefEncoderDeg = meanEnc;
        fitRefTurretDeg = meanTrue;

        // R² computation
        double ssTot = 0, ssRes = 0;
        for (int i = 0; i < captureCount; i++) {
            double predicted = fitRefTurretDeg + fitScale * (capturedEncoderDeg[i] - fitRefEncoderDeg);
            double residual = capturedTrueDeg[i] - predicted;
            ssRes += residual * residual;
            ssTot += (capturedTrueDeg[i] - meanTrue) * (capturedTrueDeg[i] - meanTrue);
        }
        fitRSquared = (Math.abs(ssTot) < 1e-9) ? 1.0 : (1.0 - ssRes / ssTot);

        fitComputed = true;
    }

    private boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private String fmt(double v) {
        if (Double.isNaN(v)) return "---";
        return String.format("%.3f", v);
    }
}
