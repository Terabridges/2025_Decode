package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

/**
 * Minimal spindex tuning OpMode focused on fast loop times.
 * - Constructs ONLY the spindex subsystem (no full Robot)
 * - No Panels/JoinedTelemetry (can be expensive)
 * - Optional decimation of telemetry/logging
 */
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "SpindexTunerLite", group = "Test")
public class SpindexTunerLite extends OpMode {

    private Spindex spindex;
    private VoltageSensor voltageSensor;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    // Tuning knobs
    public static double stepDeg = 60.0;
    public static double fineStepDeg = 5.0;
    public static double manualMaxPower = 0.2;

    // Reduce overhead when chasing faster loop times
    public static int telemetryEveryN = 5; // 1 = every loop
    public static int logEveryN = 1;       // 1 = every loop

    // Characterization (find minimum power to move in each direction)
    public static boolean enableCharacterize = true;
    public static double charMaxPower = 0.60;
    public static double charRampRatePerSec = 0.25; // power units per second
    public static double charOmegaThresholdDegPerSec = 2.0;
    public static double charPosDeltaThresholdDeg = 0.25; // per-loop position delta to consider "moving"
    public static int charConsecutiveSamples = 3;
    public static double charMinDetectPower = 0.03; // ignore tiny powers (noise) before checking motion
    public static double charMarginPower = 0.01;
    public static double charSettleSec = 0.25;

    private long loopCount = 0;
    private double lastRuntimeSec = 0.0;

    private enum CharState { IDLE, POS_RAMP, POS_SETTLE, NEG_RAMP, NEG_SETTLE, DONE }
    private CharState charState = CharState.IDLE;
    private double charPower = 0.0;
    private double charT = 0.0;
    private double minPowerPos = Double.NaN;
    private double minPowerNeg = Double.NaN;
    private double lastCharPos = 0.0;
    private int charMoveCount = 0;

    // Debug for characterization detection
    private double lastCharPosDelta = 0.0;
    private double lastCharOmegaSigned = 0.0;
    private double lastCharPosDeltaSigned = 0.0;

    @Override
    public void init() {
        spindex = new Spindex(hardwareMap, false);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Logger.recordMetadata("OpMode", "SpindexTunerLite");
        Logger.recordMetadata("Robot", "2025_Decode");

        spindex.toInit();
        Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
        spindex.useSpindexPID = true;

        telemetry.addLine("SpindexTunerLite ready");
        telemetry.addLine("A toggle PID | B hold | LB/RB step | dpad fine | triggers manual");
        telemetry.update();
    }

    @Override
    public void start() {
        spindex.toInit();
        Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
        spindex.useSpindexPID = true;
        lastRuntimeSec = getRuntime();
    }

    @Override
    public void loop() {
        loopCount++;

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Y: run stiction characterization (disables PID; uses power ramp)
        if (enableCharacterize && currentGamepad1.y && !previousGamepad1.y) {
            charState = CharState.POS_RAMP;
            charPower = 0.0;
            charT = 0.0;
            minPowerPos = Double.NaN;
            minPowerNeg = Double.NaN;
            charMoveCount = 0;
            lastCharPos = spindex.getCurrentPositionFiltered();
            spindex.useSpindexPID = false;
            Spindex.manualPower = 0.0;
        }

        // --- Controls ---
        if (currentGamepad1.a && !previousGamepad1.a) {
            spindex.useSpindexPID = !spindex.useSpindexPID;
            if (spindex.useSpindexPID) {
                Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
            }
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
            spindex.useSpindexPID = true;
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget - stepDeg);
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget + stepDeg);
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget - fineStepDeg);
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget + fineStepDeg);
        }

        // If characterization is running, it owns manualPower.
        if (charState == CharState.IDLE || charState == CharState.DONE) {
            double manual = (currentGamepad1.right_trigger - currentGamepad1.left_trigger) * manualMaxPower;
            boolean manualActive = Math.abs(manual) > 0.02;
            if (manualActive) {
                spindex.useSpindexPID = false;
                Spindex.manualPower = manual;
            } else {
                Spindex.manualPower = 0.0;

                boolean wasManual = (Math.abs(previousGamepad1.right_trigger - previousGamepad1.left_trigger) * manualMaxPower) > 0.02;
                if (wasManual) {
                    Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
                    spindex.useSpindexPID = true;
                }
            }
        }

        // --- Update controller ---
        spindex.update();

        // Loop dt (OpMode cadence) vs controller dt (spindex.getLastDt())
        double now = getRuntime();
        double loopDt = now - lastRuntimeSec;
        lastRuntimeSec = now;

        // --- Characterization state machine ---
        if (charState != CharState.IDLE && charState != CharState.DONE) {
            charT += loopDt;

            // Use unfiltered omega for detection. Filtered omega can stay tiny if heavily smoothed.
            double omega = spindex.getLastOmegaUnfilteredDegPerSec();
            double posNow = spindex.getCurrentPositionFiltered();
            double dPos = (posNow - lastCharPos);
            double posDelta = Math.abs(dPos);
            lastCharPos = posNow;
            lastCharPosDelta = posDelta;

            // Require motion in the commanded direction to avoid false triggers from noise/oscillation.
            double cmdSign = (Spindex.manualPower > 0) ? 1.0 : (Spindex.manualPower < 0) ? -1.0 : 0.0;
            lastCharOmegaSigned = omega * cmdSign;
            lastCharPosDeltaSigned = dPos * cmdSign;

            boolean powerHighEnough = Math.abs(Spindex.manualPower) >= charMinDetectPower;

            boolean movingThisSample = powerHighEnough && (
                (lastCharOmegaSigned >= charOmegaThresholdDegPerSec) ||
                (lastCharPosDeltaSigned >= charPosDeltaThresholdDeg)
            );
            if (movingThisSample) {
                charMoveCount++;
            } else {
                charMoveCount = 0;
            }

            boolean movingConfirmed = charMoveCount >= Math.max(1, charConsecutiveSamples);

            switch (charState) {
                case POS_RAMP:
                    spindex.useSpindexPID = false;
                    charPower = Math.min(charMaxPower, charPower + charRampRatePerSec * loopDt);
                    Spindex.manualPower = charPower;
                    if (movingConfirmed) {
                        minPowerPos = charPower;
                        charState = CharState.POS_SETTLE;
                        charT = 0.0;
                        charMoveCount = 0;
                        Spindex.manualPower = 0.0;
                    } else if (charPower >= charMaxPower) {
                        // Give up this direction
                        minPowerPos = Double.NaN;
                        charState = CharState.POS_SETTLE;
                        charT = 0.0;
                        charMoveCount = 0;
                        Spindex.manualPower = 0.0;
                    }
                    break;

                case POS_SETTLE:
                    spindex.useSpindexPID = false;
                    Spindex.manualPower = 0.0;
                    if (charT >= charSettleSec) {
                        charState = CharState.NEG_RAMP;
                        charPower = 0.0;
                        charT = 0.0;
                        charMoveCount = 0;
                    }
                    break;

                case NEG_RAMP:
                    spindex.useSpindexPID = false;
                    charPower = Math.min(charMaxPower, charPower + charRampRatePerSec * loopDt);
                    Spindex.manualPower = -charPower;
                    if (movingConfirmed) {
                        minPowerNeg = charPower;
                        charState = CharState.NEG_SETTLE;
                        charT = 0.0;
                        charMoveCount = 0;
                        Spindex.manualPower = 0.0;
                    } else if (charPower >= charMaxPower) {
                        minPowerNeg = Double.NaN;
                        charState = CharState.NEG_SETTLE;
                        charT = 0.0;
                        charMoveCount = 0;
                        Spindex.manualPower = 0.0;
                    }
                    break;

                case NEG_SETTLE:
                    spindex.useSpindexPID = false;
                    Spindex.manualPower = 0.0;
                    if (charT >= charSettleSec) {
                        // Apply suggested kS values automatically (if both were measured)
                        if (!Double.isNaN(minPowerPos)) {
                            Spindex.kSPos = minPowerPos + charMarginPower;
                        }
                        if (!Double.isNaN(minPowerNeg)) {
                            Spindex.kSNeg = minPowerNeg + charMarginPower;
                        }
                        charState = CharState.DONE;
                        // Hold current after characterization
                        Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
                        spindex.useSpindexPID = true;
                    }
                    break;
            }
        }

        // --- Telemetry (decimated) ---
        int teleN = Math.max(1, telemetryEveryN);
        if ((loopCount % teleN) == 0) {
            telemetry.addData("LoopDt (s)", loopDt);
            telemetry.addData("CtrlDt (s)", spindex.getLastDt());
            telemetry.addData("Batt (V)", voltageSensor.getVoltage());
            telemetry.addData("PID Enabled", spindex.useSpindexPID);
            telemetry.addData("Raw Deg", spindex.getLastRawDeg());
            telemetry.addData("Pos (deg)", spindex.getCurrentPositionFiltered());
            telemetry.addData("TargetRaw", spindex.getTargetPosition());
            telemetry.addData("Err (deg)", spindex.getLastErrorDeg());
            telemetry.addData("OmegaFilt (deg/s)", spindex.getLastOmegaDegPerSec());
            telemetry.addData("OmegaUnf (deg/s)", spindex.getLastOmegaUnfilteredDegPerSec());
            telemetry.addData("Power", spindex.getLastPower());

            telemetry.addData("CharState", charState);
            telemetry.addData("CharPower", charPower);
            telemetry.addData("CharMoveCount", charMoveCount);
            telemetry.addData("CharOmegaSigned", lastCharOmegaSigned);
            telemetry.addData("CharPosDeltaSigned", lastCharPosDeltaSigned);
            telemetry.addData("MinPower+", minPowerPos);
            telemetry.addData("MinPower-", minPowerNeg);
            telemetry.update();
        }

        // --- PsiKit outputs (decimated) ---
        int logN = Math.max(1, logEveryN);
        if ((loopCount % logN) == 0) {
            Logger.recordOutput("Spindex/LoopDtSec", loopDt);
            Logger.recordOutput("Spindex/CtrlDtSec", spindex.getLastDt());
            Logger.recordOutput("Spindex/BattV", voltageSensor.getVoltage());
            Logger.recordOutput("Spindex/PidEnabled", spindex.useSpindexPID);
            Logger.recordOutput("Spindex/ManualPower", Spindex.manualPower);

            Logger.recordOutput("Spindex/EncVolts", spindex.getLastVoltage());
            Logger.recordOutput("Spindex/RawDeg", spindex.getLastRawDeg());
            Logger.recordOutput("Spindex/PosUnwrappedDeg", spindex.getCurrentPosition());
            Logger.recordOutput("Spindex/PosFilteredDeg", spindex.getCurrentPositionFiltered());

            Logger.recordOutput("Spindex/TargetRawDeg", spindex.getTargetPosition());
            Logger.recordOutput("Spindex/TargetUnwrappedDeg", spindex.getTargetPositionUnwrapped());
            Logger.recordOutput("Spindex/ErrorDeg", spindex.getLastErrorDeg());

            Logger.recordOutput("Spindex/OmegaFiltDegPerSec", spindex.getLastOmegaDegPerSec());
            Logger.recordOutput("Spindex/OmegaUnfDegPerSec", spindex.getLastOmegaUnfilteredDegPerSec());
            Logger.recordOutput("Spindex/OmegaCmdDegPerSec", spindex.getLastOmegaCmdDegPerSec());
            Logger.recordOutput("Spindex/OmegaLimitDegPerSec", spindex.getLastOmegaLimitDegPerSec());
            Logger.recordOutput("Spindex/VelErrDegPerSec", spindex.getLastVelErrDegPerSec());
            Logger.recordOutput("Spindex/IPower", spindex.getLastIPower());
            Logger.recordOutput("Spindex/IInZone", spindex.getLastIInZone());
            Logger.recordOutput("Spindex/ISaturated", spindex.getLastISaturated());

            Logger.recordOutput("Spindex/Power", spindex.getLastPower());

            Logger.recordOutput("Spindex/Char/State", charState.toString());
            Logger.recordOutput("Spindex/Char/RampPower", charPower);
            Logger.recordOutput("Spindex/Char/MoveCount", charMoveCount);
            Logger.recordOutput("Spindex/Char/PosDeltaDeg", lastCharPosDelta);
            Logger.recordOutput("Spindex/Char/OmegaSignedDegPerSec", lastCharOmegaSigned);
            Logger.recordOutput("Spindex/Char/PosDeltaSignedDeg", lastCharPosDeltaSigned);
            Logger.recordOutput("Spindex/Char/MinPowerPos", minPowerPos);
            Logger.recordOutput("Spindex/Char/MinPowerNeg", minPowerNeg);
            Logger.recordOutput("Spindex/Char/kSPosSuggested", Spindex.kSPos);
            Logger.recordOutput("Spindex/Char/kSNegSuggested", Spindex.kSNeg);
        }
    }

    private static double wrap0To360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }
}
