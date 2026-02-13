package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.Locale;

/**
 * Auto-tunes the spindex by:
 * 1) Characterizing directional stiction (kSPos/kSNeg)
 * 2) Sweeping a small grid of (kPos, kVel) and scoring step response
 *
 * This is intentionally conservative: it won't magically produce perfect gains,
 * but it will quickly find a "better" region on a real robot.
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "SpindexAutoTuner", group = "Test")
public class SpindexAutoTuner extends OpMode {

    // --- User knobs ---
    public static boolean runStictionChar = true;

    // Which controller to tune
    // 0 = cascaded controller (sweeps Spindex.kPos / Spindex.kVel)
    // 1 = motion-profile controller (sweeps Spindex.profKp / Spindex.profKv)
    public static int tuneControllerMode = 0;

    public static double charSettleSec = 0.25;
    public static double charRampRatePerSec = 0.30;
    public static double charMaxPower = 0.35;
    public static double charOmegaThreshDegPerSec = 25.0;
    public static double charPosDeltaThreshDeg = 0.8;
    public static int charMoveCountRequired = 4;
    public static double charMarginPower = 0.015;

    public static double stepDeg = 60.0;
    public static double restSec = 0.30;
    public static double testDurationSec = 2.5;

    public static double settleTolDeg = 4.0;
    public static double settleVelTolDegPerSec = 80.0;
    public static double settleHoldSec = 0.25;

    // Sweep scales (applied to starting gains)
    // For controllerMode=0 these scale (kPos, kVel). For controllerMode=1 these scale (profKp, profKv).
    public static double[] kPosScales = new double[]{0.6, 1.0, 1.6};
    public static double[] kVelScales = new double[]{0.7, 1.0, 1.4};

    // Score weights
    public static double wSettleTime = 1.0;
    public static double wOvershoot = 0.5;
    public static double wIAE = 0.01;

    // --- Internal state ---
    private Spindex spindex;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private enum State {
        WAIT_A,
        CHAR_POS_SETTLE,
        CHAR_POS_RAMP,
        CHAR_NEG_SETTLE,
        CHAR_NEG_RAMP,
        SWEEP_PREP,
        SWEEP_REST,
        SWEEP_STEP_POS,
        SWEEP_REST_2,
        SWEEP_STEP_NEG,
        DONE
    }

    private State state = State.WAIT_A;

    private double lastLoopSec = 0.0;

    // Stiction characterization
    private double charT = 0.0;
    private double rampPower = 0.0;
    private double lastPosForChar = 0.0;
    private double minPowerPos = Double.NaN;
    private double minPowerNeg = Double.NaN;
    private int moveCount = 0;

    // Sweep
    private double baseKPos;
    private double baseKVel;
    private int iPos = 0;
    private int iVel = 0;
    private boolean appliedPair = false;

    private double bestScore = Double.POSITIVE_INFINITY;
    private double bestKPos = 0.0;
    private double bestKVel = 0.0;

    // Step test metrics
    private static class StepMetrics {
        double iae;
        double overshoot;
        double settleTime;
    }

    private double stepStartT = 0.0;
    private double stableT = 0.0;
    private double maxOvershoot = 0.0;
    private double iae = 0.0;
    private boolean settled = false;
    private double settledTime = 0.0;
    private int stepSign = 1;

    private double scorePos = 0.0;
    private double scoreNeg = 0.0;

    @Override
    public void init() {
        spindex = new Spindex(hardwareMap, false);

        Logger.recordMetadata("OpMode", "SpindexAutoTuner");
        Logger.recordMetadata("Robot", "2025_Decode");

        telemetry.addLine("SpindexAutoTuner ready");
        telemetry.addLine("Press A to start (robot will move spindex automatically)");
        telemetry.update();

        lastLoopSec = getRuntime();
    }

    @Override
    public void start() {
        spindex.toInit();
        Spindex.controllerMode = tuneControllerMode;
        spindex.useSpindexPID = true;
        Spindex.manualPower = 0.0;

        if (tuneControllerMode == 1) {
            baseKPos = Spindex.profKp;
            baseKVel = Spindex.profKv;
        } else {
            baseKPos = Spindex.kPos;
            baseKVel = Spindex.kVel;
        }

        bestKPos = baseKPos;
        bestKVel = baseKVel;

        lastPosForChar = spindex.getCurrentPositionFiltered();
        lastLoopSec = getRuntime();
    }

    @Override
    public void loop() {
        gamepadUpdate();

        double now = getRuntime();
        double dt = now - lastLoopSec;
        if (dt <= 1e-4) dt = 1e-4;
        lastLoopSec = now;

        spindex.update();

        Logger.recordOutput("AutoTune/State", state.name());
        Logger.recordOutput("AutoTune/ControllerMode", Spindex.controllerMode);
        Logger.recordOutput("AutoTune/kPos", Spindex.kPos);
        Logger.recordOutput("AutoTune/kVel", Spindex.kVel);
        Logger.recordOutput("AutoTune/profKp", Spindex.profKp);
        Logger.recordOutput("AutoTune/profKv", Spindex.profKv);
        Logger.recordOutput("AutoTune/ErrorDeg", spindex.getLastErrorDeg());
        Logger.recordOutput("AutoTune/OmegaDegPerSec", spindex.getLastOmegaDegPerSec());
        Logger.recordOutput("AutoTune/Power", spindex.getLastPower());

        switch (state) {
            case WAIT_A:
                spindex.useSpindexPID = true;
                Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());

                telemetry.addLine("Press A to start auto-tune");
                telemetry.addData("TuneMode", tuneControllerMode);
                telemetry.addData("PosRaw", "%.1f", spindex.getLastRawDeg());
                telemetry.addData("PosUnwrap", "%.1f", spindex.getCurrentPositionFiltered());
                if (tuneControllerMode == 1) {
                    telemetry.addData("profKp/profKv", "%.5f / %.6f", Spindex.profKp, Spindex.profKv);
                } else {
                    telemetry.addData("kPos/kVel", "%.4f / %.6f", Spindex.kPos, Spindex.kVel);
                }
                telemetry.update();

                if (current.a && !previous.a) {
                    if (runStictionChar) {
                        beginChar(dt);
                        state = State.CHAR_POS_SETTLE;
                    } else {
                        state = State.SWEEP_PREP;
                    }
                }
                break;

            case CHAR_POS_SETTLE:
                spindex.useSpindexPID = false;
                Spindex.manualPower = 0.0;
                charT += dt;
                if (charT >= charSettleSec) {
                    charT = 0.0;
                    rampPower = 0.0;
                    moveCount = 0;
                    lastPosForChar = spindex.getCurrentPositionFiltered();
                    state = State.CHAR_POS_RAMP;
                }
                logChar();
                break;

            case CHAR_POS_RAMP:
                spindex.useSpindexPID = false;
                rampPower = clamp(rampPower + charRampRatePerSec * dt, 0.0, charMaxPower);
                Spindex.manualPower = rampPower;

                if (detectMotion(+1)) {
                    moveCount++;
                } else {
                    moveCount = 0;
                }

                if (moveCount >= charMoveCountRequired) {
                    minPowerPos = rampPower;
                    Spindex.manualPower = 0.0;
                    beginChar(dt);
                    state = State.CHAR_NEG_SETTLE;
                }
                logChar();
                break;

            case CHAR_NEG_SETTLE:
                spindex.useSpindexPID = false;
                Spindex.manualPower = 0.0;
                charT += dt;
                if (charT >= charSettleSec) {
                    charT = 0.0;
                    rampPower = 0.0;
                    moveCount = 0;
                    lastPosForChar = spindex.getCurrentPositionFiltered();
                    state = State.CHAR_NEG_RAMP;
                }
                logChar();
                break;

            case CHAR_NEG_RAMP:
                spindex.useSpindexPID = false;
                rampPower = clamp(rampPower + charRampRatePerSec * dt, 0.0, charMaxPower);
                Spindex.manualPower = -rampPower;

                if (detectMotion(-1)) {
                    moveCount++;
                } else {
                    moveCount = 0;
                }

                if (moveCount >= charMoveCountRequired) {
                    minPowerNeg = rampPower;
                    Spindex.manualPower = 0.0;

                    // Apply suggested kS
                    if (!Double.isNaN(minPowerPos)) {
                        Spindex.kSPos = minPowerPos + charMarginPower;
                    }
                    if (!Double.isNaN(minPowerNeg)) {
                        Spindex.kSNeg = minPowerNeg + charMarginPower;
                    }

                    spindex.useSpindexPID = true;
                    state = State.SWEEP_PREP;
                }
                logChar();
                break;

            case SWEEP_PREP:
                // Reset to baseline before sweep
                if (tuneControllerMode == 1) {
                    Spindex.controllerMode = 1;
                    Spindex.profKp = baseKPos;
                    Spindex.profKv = baseKVel;
                } else {
                    Spindex.controllerMode = 0;
                    Spindex.kPos = baseKPos;
                    Spindex.kVel = baseKVel;
                }

                iPos = 0;
                iVel = 0;
                bestScore = Double.POSITIVE_INFINITY;
                bestKPos = baseKPos;
                bestKVel = baseKVel;
                appliedPair = false;

                state = State.SWEEP_REST;
                charT = 0.0;
                break;

            case SWEEP_REST:
                // Apply candidate gains once at the start of the pair
                if (!appliedPair) {
                    double candKPos = baseKPos * kPosScales[iPos];
                    double candKVel = baseKVel * kVelScales[iVel];
                    if (tuneControllerMode == 1) {
                        Spindex.controllerMode = 1;
                        Spindex.profKp = candKPos;
                        Spindex.profKv = candKVel;
                    } else {
                        Spindex.controllerMode = 0;
                        Spindex.kPos = candKPos;
                        Spindex.kVel = candKVel;
                    }
                    appliedPair = true;

                    // Hold target at current
                    spindex.useSpindexPID = true;
                    Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());

                    Logger.recordOutput("AutoTune/CandKPos", candKPos);
                    Logger.recordOutput("AutoTune/CandKVel", candKVel);
                }

                charT += dt;
                if (charT >= restSec) {
                    beginStep(+1);
                    state = State.SWEEP_STEP_POS;
                }
                break;

            case SWEEP_STEP_POS:
                runStep(dt);
                if (stepDone()) {
                    StepMetrics m = finalizeStep();
                    scorePos = score(m);
                    beginRest();
                    state = State.SWEEP_REST_2;
                }
                break;

            case SWEEP_REST_2:
                charT += dt;
                if (charT >= restSec) {
                    beginStep(-1);
                    state = State.SWEEP_STEP_NEG;
                }
                break;

            case SWEEP_STEP_NEG:
                runStep(dt);
                if (stepDone()) {
                    StepMetrics m = finalizeStep();
                    scoreNeg = score(m);

                    double totalScore = scorePos + scoreNeg;
                    Logger.recordOutput("AutoTune/PairScore", totalScore);

                    if (totalScore < bestScore) {
                        bestScore = totalScore;
                        if (tuneControllerMode == 1) {
                            bestKPos = Spindex.profKp;
                            bestKVel = Spindex.profKv;
                        } else {
                            bestKPos = Spindex.kPos;
                            bestKVel = Spindex.kVel;
                        }
                    }

                    advancePairOrDone();
                }
                break;

            case DONE:
                // Apply best and hold
                if (tuneControllerMode == 1) {
                    Spindex.controllerMode = 1;
                    Spindex.profKp = bestKPos;
                    Spindex.profKv = bestKVel;
                } else {
                    Spindex.controllerMode = 0;
                    Spindex.kPos = bestKPos;
                    Spindex.kVel = bestKVel;
                }
                spindex.useSpindexPID = true;
                Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());

                Logger.recordOutput("AutoTune/BestScore", bestScore);
                Logger.recordOutput("AutoTune/BestKPos", bestKPos);
                Logger.recordOutput("AutoTune/BestKVel", bestKVel);
                Logger.recordOutput("AutoTune/MinPowerPos", minPowerPos);
                Logger.recordOutput("AutoTune/MinPowerNeg", minPowerNeg);

                telemetry.addLine("Auto-tune complete");
                telemetry.addData("BestScore", "%.3f", bestScore);
                if (tuneControllerMode == 1) {
                    telemetry.addData("Best profKp", "%.5f", bestKPos);
                    telemetry.addData("Best profKv", "%.6f", bestKVel);
                } else {
                    telemetry.addData("Best kPos", "%.4f", bestKPos);
                    telemetry.addData("Best kVel", "%.6f", bestKVel);
                }
                telemetry.addData("kSPos/kSNeg", "%.3f / %.3f", Spindex.kSPos, Spindex.kSNeg);
                telemetry.addLine("Press stop to exit");
                telemetry.update();
                break;
        }

        telemetry.addData("State", state.name());
        if (tuneControllerMode == 1) {
            telemetry.addData("profKp/profKv", "%.5f / %.6f", Spindex.profKp, Spindex.profKv);
        } else {
            telemetry.addData("kPos/kVel", "%.4f / %.6f", Spindex.kPos, Spindex.kVel);
        }
        telemetry.addData("Err", "%.2f", spindex.getLastErrorDeg());
        telemetry.addData("Omega", "%.1f", spindex.getLastOmegaDegPerSec());
        telemetry.update();
    }

    private void gamepadUpdate() {
        previous.copy(current);
        current.copy(gamepad1);
    }

    private void beginChar(double dt) {
        charT = 0.0;
        moveCount = 0;
        rampPower = 0.0;
        lastPosForChar = spindex.getCurrentPositionFiltered();
        Logger.recordOutput("AutoTune/CharReset", dt);
    }

    private void logChar() {
        double posNow = spindex.getCurrentPositionFiltered();
        double posDelta = posNow - lastPosForChar;
        lastPosForChar = posNow;

        Logger.recordOutput("AutoTune/CharRampPower", rampPower);
        Logger.recordOutput("AutoTune/CharMoveCount", moveCount);
        Logger.recordOutput("AutoTune/CharPosDeltaDeg", posDelta);
        Logger.recordOutput("AutoTune/CharOmegaUnfDegPerSec", spindex.getLastOmegaUnfilteredDegPerSec());
        Logger.recordOutput("AutoTune/MinPowerPos", minPowerPos);
        Logger.recordOutput("AutoTune/MinPowerNeg", minPowerNeg);
    }

    private boolean detectMotion(int directionSign) {
        double omega = spindex.getLastOmegaUnfilteredDegPerSec();
        double posNow = spindex.getCurrentPositionFiltered();
        double posDelta = posNow - lastPosForChar;
        lastPosForChar = posNow;

        boolean omegaMove = (omega * directionSign) > charOmegaThreshDegPerSec;
        boolean posMove = (posDelta * directionSign) > charPosDeltaThreshDeg;

        Logger.recordOutput("AutoTune/CharOmegaSigned", omega * directionSign);
        Logger.recordOutput("AutoTune/CharPosDeltaSigned", posDelta * directionSign);

        return omegaMove || posMove;
    }

    private void beginRest() {
        charT = 0.0;
        spindex.useSpindexPID = true;
        Spindex.spindexTarget = wrap0To360(spindex.getLastRawDeg());
    }

    private void beginStep(int sign) {
        stepSign = sign;
        stepStartT = getRuntime();
        stableT = 0.0;
        maxOvershoot = 0.0;
        iae = 0.0;
        settled = false;
        settledTime = testDurationSec;

        double rawNow = spindex.getLastRawDeg();
        double targetRaw = wrap0To360(rawNow + sign * stepDeg);
        Spindex.spindexTarget = targetRaw;

        Logger.recordOutput("AutoTune/StepSign", sign);
        Logger.recordOutput("AutoTune/StepTargetRaw", targetRaw);
    }

    private void runStep(double dt) {
        double error = spindex.getLastErrorDeg();
        double omega = spindex.getLastOmegaDegPerSec();
        double elapsed = getRuntime() - stepStartT;

        iae += Math.abs(error) * dt;

        // Signed overshoot: how far past target in the step direction.
        // If stepSign=+1: overshoot is max(pos-target)
        // If stepSign=-1: overshoot is max(target-pos)
        double pos = spindex.getCurrentPositionFiltered();
        double target = spindex.getTargetPositionUnwrapped();
        double signedOvershoot = (pos - target) * stepSign;
        if (signedOvershoot > maxOvershoot) maxOvershoot = signedOvershoot;

        boolean inBand = Math.abs(error) <= settleTolDeg && Math.abs(omega) <= settleVelTolDegPerSec;
        if (inBand) {
            stableT += dt;
            if (!settled && stableT >= settleHoldSec) {
                settled = true;
                settledTime = Math.max(0.0, elapsed - settleHoldSec);
            }
        } else {
            stableT = 0.0;
        }

        Logger.recordOutput("AutoTune/StepElapsed", elapsed);
        Logger.recordOutput("AutoTune/StepIAE", iae);
        Logger.recordOutput("AutoTune/StepOvershoot", maxOvershoot);
        Logger.recordOutput("AutoTune/StepSettled", settled);
        Logger.recordOutput("AutoTune/StepSettledTime", settledTime);
    }

    private boolean stepDone() {
        return (getRuntime() - stepStartT) >= testDurationSec;
    }

    private StepMetrics finalizeStep() {
        StepMetrics m = new StepMetrics();
        m.iae = iae;
        m.overshoot = maxOvershoot;
        m.settleTime = settledTime;

        Logger.recordOutput("AutoTune/MetricIAE", m.iae);
        Logger.recordOutput("AutoTune/MetricOvershoot", m.overshoot);
        Logger.recordOutput("AutoTune/MetricSettleTime", m.settleTime);

        return m;
    }

    private double score(StepMetrics m) {
        return wSettleTime * m.settleTime + wOvershoot * Math.abs(m.overshoot) + wIAE * m.iae;
    }

    private void advancePairOrDone() {
        // Advance kVel fastest, then kPos
        iVel++;
        if (iVel >= kVelScales.length) {
            iVel = 0;
            iPos++;
        }

        appliedPair = false;
        beginRest();

        if (iPos >= kPosScales.length) {
            state = State.DONE;
        } else {
            state = State.SWEEP_REST;
        }

        Logger.recordOutput("AutoTune/BestScoreSoFar", bestScore);
        Logger.recordOutput("AutoTune/BestKPosSoFar", bestKPos);
        Logger.recordOutput("AutoTune/BestKVelSoFar", bestKVel);
    }

    private static double wrap0To360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
