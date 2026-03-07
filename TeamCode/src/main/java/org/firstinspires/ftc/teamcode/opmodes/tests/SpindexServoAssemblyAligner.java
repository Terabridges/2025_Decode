package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "SpindexServoAssemblyAligner", group = "Test")
public class SpindexServoAssemblyAligner extends OpMode {

    public enum Mode {
        ALIGN,
        VERIFY,
        CHARACTERIZE
    }

    public enum CharacterizeServo {
        LEFT,
        RIGHT
    }

    public static double centerPos = 0.50;
    public static double targetPos = 0.50;
    public static double minPos = 0.00;
    public static double maxPos = 1.00;
    public static double stickRatePerSec = 0.35;
    public static double smallStep = 0.0025;
    public static double largeStep = 0.01;

    public static boolean invertRight = false;
    public static double rightOffset = 0.012;

    public static double verifySweepMin = 0.15;
    public static double verifySweepMax = 0.85;
    public static double verifySweepRatePerSec = 0.20;

    public static int maxCharacterizeSamples = 20;
    public static double compareCommandTolerance = 0.01;
    public static double pwmMinUs = 500.0;
    public static double pwmMaxUs = 2500.0;
    public static double rightOffsetSmallStep = 0.001;
    public static double rightOffsetLargeStep = 0.005;

    private Servo spindexL;
    private Servo spindexR;
    private AbsoluteAnalogEncoder spindexEnc;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private Mode selectedMode = Mode.ALIGN;
    private boolean modeRunning = false;
    private boolean leftEnabled = true;
    private boolean rightEnabled = true;
    private boolean verifyAutoSweep = false;
    private double verifySweepDirection = 1.0;
    private CharacterizeServo characterizeServo = CharacterizeServo.LEFT;

    private final double[] leftCmdSamples = new double[64];
    private final double[] leftEncSamples = new double[64];
    private int leftSampleCount = 0;
    private final double[] rightCmdSamples = new double[64];
    private final double[] rightEncSamples = new double[64];
    private int rightSampleCount = 0;

    private int compareMatchedCount = 0;
    private double compareAvgSignedEncErr = Double.NaN;
    private double compareAvgAbsEncErr = Double.NaN;
    private double compareMaxAbsEncErr = Double.NaN;
    private double suggestedRightOffsetDelta = Double.NaN;
    private double leftDegPerCommand = Double.NaN;
    private double rightDegPerCommand = Double.NaN;

    private AnalogInput floodgateAnalog;
    private double floodgateCurrent = Double.NaN;

    private long lastLoopNs;

    @Override
    public void init() {
        spindexL = hardwareMap.get(Servo.class, "spindexL");
        spindexR = hardwareMap.get(Servo.class, "spindexR");

        applyPwmRange(spindexL);
        applyPwmRange(spindexR);

        try {
            AnalogInput analog = hardwareMap.get(AnalogInput.class, "spindexAnalog");
            //spindexEnc = new AbsoluteAnalogEncoder(analog, 3.3, 29, 1.17);
            spindexEnc = new AbsoluteAnalogEncoder(analog, 3.3, 0, 2.05);
            spindexEnc.setInverted(false);
        } catch (Exception ignored) {
            spindexEnc = null;
        }

        floodgateAnalog = hardwareMap.get(AnalogInput.class, "floodgate");
        floodgateCurrent = floodgateAnalog.getVoltage() / 3.3 * 80.0;

        writeServo(spindexL, false, computeLeftCmd());
        writeServo(spindexR, false, computeRightCmd());

        lastLoopNs = System.nanoTime();
        targetPos = clampTargetToSharedRange(targetPos);
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);
        floodgateCurrent = floodgateAnalog.getVoltage() / 3.3 * 80.0;

        long nowNs = System.nanoTime();
        double dtSec = Math.max(0.0, (nowNs - lastLoopNs) / 1e9);
        lastLoopNs = nowNs;

        if (!modeRunning && edge(current.start, previous.start)) {
            selectedMode = nextMode(selectedMode);
            verifyAutoSweep = false;
        }

        boolean startedThisLoop = false;
        if (!modeRunning && edge(current.a, previous.a)) {
            modeRunning = true;
            startedThisLoop = true;
        }

        if (edge(current.right_stick_button, previous.right_stick_button)) {
            modeRunning = false;
        }

        if (modeRunning && edge(current.back, previous.back)) {
            targetPos = clampTargetToSharedRange(centerPos);
        }

        if (modeRunning) {
            boolean rtNow = current.right_trigger > 0.6;
            boolean rtPrev = previous.right_trigger > 0.6;
            boolean ltNow = current.left_trigger > 0.6;
            boolean ltPrev = previous.left_trigger > 0.6;

            if (edge(current.dpad_up, previous.dpad_up)) rightOffset += rightOffsetSmallStep;
            if (edge(current.dpad_down, previous.dpad_down)) rightOffset -= rightOffsetSmallStep;
            if (edge(rtNow, rtPrev)) rightOffset += rightOffsetLargeStep;
            if (edge(ltNow, ltPrev)) rightOffset -= rightOffsetLargeStep;

            rightOffset = clamp(rightOffset, getRightOffsetMin(), getRightOffsetMax());
            targetPos = clampTargetToSharedRange(targetPos);
        }

        if (modeRunning && !startedThisLoop) {
            if (selectedMode == Mode.ALIGN) {
                runAlignMode(dtSec);
            } else if (selectedMode == Mode.VERIFY) {
                runVerifyMode(dtSec);
            } else {
                runCharacterizeMode(dtSec);
            }
        } else {
            leftEnabled = false;
            rightEnabled = false;
            verifyAutoSweep = false;
        }

        writeServo(spindexL, leftEnabled, computeLeftCmd());
        writeServo(spindexR, rightEnabled, computeRightCmd());

        showTelemetry();
    }

    @Override
    public void stop() {
        writeServo(spindexL, false, computeLeftCmd());
        writeServo(spindexR, false, computeRightCmd());
    }

    private void runAlignMode(double dtSec) {
        targetPos += (-current.left_stick_y) * stickRatePerSec * dtSec;

        if (edge(current.dpad_right, previous.dpad_right)) targetPos += smallStep;
        if (edge(current.dpad_left, previous.dpad_left)) targetPos -= smallStep;
        if (edge(current.right_bumper, previous.right_bumper)) targetPos += largeStep;
        if (edge(current.left_bumper, previous.left_bumper)) targetPos -= largeStep;
        if (edge(current.a, previous.a)) targetPos = centerPos;

        if (edge(current.x, previous.x)) leftEnabled = !leftEnabled;
        if (edge(current.b, previous.b)) rightEnabled = !rightEnabled;
        if (edge(current.y, previous.y)) {
            leftEnabled = true;
            rightEnabled = true;
        }

        targetPos = clampTargetToSharedRange(targetPos);
    }

    private void runVerifyMode(double dtSec) {
        leftEnabled = true;
        rightEnabled = true;

        if (edge(current.x, previous.x)) {
            verifyAutoSweep = !verifyAutoSweep;
        }
        if (edge(current.y, previous.y)) {
            verifySweepDirection *= -1.0;
        }
        if (edge(current.a, previous.a)) {
            verifyAutoSweep = false;
            targetPos = centerPos;
        }

        if (verifyAutoSweep) {
            targetPos += verifySweepDirection * verifySweepRatePerSec * dtSec;
            double lo = Math.min(verifySweepMin, verifySweepMax);
            double hi = Math.max(verifySweepMin, verifySweepMax);

            if (targetPos >= hi) {
                targetPos = hi;
                verifySweepDirection = -1.0;
            } else if (targetPos <= lo) {
                targetPos = lo;
                verifySweepDirection = 1.0;
            }
        } else {
            targetPos += (-current.left_stick_y) * stickRatePerSec * dtSec;
            if (edge(current.dpad_right, previous.dpad_right)) targetPos += smallStep;
            if (edge(current.dpad_left, previous.dpad_left)) targetPos -= smallStep;
            if (edge(current.right_bumper, previous.right_bumper)) targetPos += largeStep;
            if (edge(current.left_bumper, previous.left_bumper)) targetPos -= largeStep;
        }

        targetPos = clampTargetToSharedRange(targetPos);
    }

    private void runCharacterizeMode(double dtSec) {
        if (edge(current.y, previous.y)) {
            characterizeServo = (characterizeServo == CharacterizeServo.LEFT)
                    ? CharacterizeServo.RIGHT
                    : CharacterizeServo.LEFT;
        }

        leftEnabled = characterizeServo == CharacterizeServo.LEFT;
        rightEnabled = characterizeServo == CharacterizeServo.RIGHT;

        targetPos += (-current.left_stick_y) * stickRatePerSec * dtSec;
        if (edge(current.dpad_right, previous.dpad_right)) targetPos += smallStep;
        if (edge(current.dpad_left, previous.dpad_left)) targetPos -= smallStep;
        if (edge(current.right_bumper, previous.right_bumper)) targetPos += largeStep;
        if (edge(current.left_bumper, previous.left_bumper)) targetPos -= largeStep;
        if (edge(current.back, previous.back)) targetPos = centerPos;
        targetPos = clampTargetToSharedRange(targetPos);

        if (edge(current.a, previous.a)) {
            captureSample();
        }

        if (edge(current.b, previous.b)) {
            clearActiveSamples();
        }

        if (edge(current.x, previous.x)) {
            computeCompareStats();
        }
    }

    private void captureSample() {
        if (spindexEnc == null) return;

        int cap = clampSampleCap();
        double encoderDeg = spindexEnc.getCurrentPosition();
        if (characterizeServo == CharacterizeServo.LEFT) {
            if (leftSampleCount < cap) {
                leftCmdSamples[leftSampleCount] = computeLeftCmd();
                leftEncSamples[leftSampleCount] = encoderDeg;
                leftSampleCount++;
            }
        } else {
            if (rightSampleCount < cap) {
                rightCmdSamples[rightSampleCount] = computeRightCmd();
                rightEncSamples[rightSampleCount] = encoderDeg;
                rightSampleCount++;
            }
        }
    }

    private void clearActiveSamples() {
        if (characterizeServo == CharacterizeServo.LEFT) {
            leftSampleCount = 0;
        } else {
            rightSampleCount = 0;
        }
    }

    private void computeCompareStats() {
        compareMatchedCount = 0;
        compareAvgSignedEncErr = Double.NaN;
        compareAvgAbsEncErr = Double.NaN;
        compareMaxAbsEncErr = Double.NaN;
        suggestedRightOffsetDelta = Double.NaN;
        leftDegPerCommand = Double.NaN;
        rightDegPerCommand = Double.NaN;

        if (leftSampleCount == 0 || rightSampleCount == 0) return;

        double tolerance = Math.max(0.0, compareCommandTolerance);
        double sumSigned = 0.0;
        double sumAbs = 0.0;
        double maxAbs = 0.0;

        for (int li = 0; li < leftSampleCount; li++) {
            double lCmd = leftCmdSamples[li];
            double lEnc = leftEncSamples[li];

            int bestRi = -1;
            double bestCmdDiff = Double.POSITIVE_INFINITY;

            for (int ri = 0; ri < rightSampleCount; ri++) {
                double diff = Math.abs(rightCmdSamples[ri] - lCmd);
                if (diff < bestCmdDiff) {
                    bestCmdDiff = diff;
                    bestRi = ri;
                }
            }

            if (bestRi >= 0 && bestCmdDiff <= tolerance) {
                double err = wrapSignedDeg(rightEncSamples[bestRi] - lEnc);
                compareMatchedCount++;
                sumSigned += err;
                double abs = Math.abs(err);
                sumAbs += abs;
                maxAbs = Math.max(maxAbs, abs);
            }
        }

        if (compareMatchedCount > 0) {
            compareAvgSignedEncErr = sumSigned / compareMatchedCount;
            compareAvgAbsEncErr = sumAbs / compareMatchedCount;
            compareMaxAbsEncErr = maxAbs;

            leftDegPerCommand = estimateSlopeDegPerPos(leftCmdSamples, leftEncSamples, leftSampleCount);
            rightDegPerCommand = estimateSlopeDegPerPos(rightCmdSamples, rightEncSamples, rightSampleCount);
            double avgSlope = Double.NaN;
            if (!Double.isNaN(leftDegPerCommand) && !Double.isNaN(rightDegPerCommand)) {
                avgSlope = 0.5 * (leftDegPerCommand + rightDegPerCommand);
            } else if (!Double.isNaN(leftDegPerCommand)) {
                avgSlope = leftDegPerCommand;
            } else if (!Double.isNaN(rightDegPerCommand)) {
                avgSlope = rightDegPerCommand;
            }

            if (!Double.isNaN(avgSlope) && Math.abs(avgSlope) > 1e-6) {
                suggestedRightOffsetDelta = -(compareAvgSignedEncErr / avgSlope);
            }
        }
    }

    private double estimateSlopeDegPerPos(double[] cmdSamples, double[] encSamples, int count) {
        if (count < 2) return Double.NaN;

        double sumX = 0.0;
        double sumY = 0.0;
        for (int i = 0; i < count; i++) {
            sumX += cmdSamples[i];
            sumY += encSamples[i];
        }
        double meanX = sumX / count;
        double meanY = sumY / count;

        double sxx = 0.0;
        double sxy = 0.0;
        for (int i = 0; i < count; i++) {
            double dx = cmdSamples[i] - meanX;
            double dy = wrapSignedDeg(encSamples[i] - meanY);
            sxx += dx * dx;
            sxy += dx * dy;
        }

        if (Math.abs(sxx) < 1e-9) return Double.NaN;
        return sxy / sxx;
    }

    private int clampSampleCap() {
        int cap = Math.max(1, Math.min(maxCharacterizeSamples, leftCmdSamples.length));
        maxCharacterizeSamples = cap;
        return cap;
    }

    private Mode nextMode(Mode currentMode) {
        if (currentMode == Mode.ALIGN) return Mode.VERIFY;
        if (currentMode == Mode.VERIFY) return Mode.CHARACTERIZE;
        return Mode.ALIGN;
    }

    private double computeLeftCmd() {
        return clamp(targetPos, minPos, maxPos);
    }

    private double computeRightCmd() {
        double base = invertRight ? (1.0 - targetPos) : targetPos;
        return clamp(base + rightOffset, minPos, maxPos);
    }

    private double clampTargetToSharedRange(double requestedBasePos) {
        double sharedMin = Math.max(minPos, invertRight ? (1.0 - maxPos) + rightOffset : minPos - rightOffset);
        double sharedMax = Math.min(maxPos, invertRight ? (1.0 - minPos) + rightOffset : maxPos - rightOffset);
        return clamp(requestedBasePos, sharedMin, sharedMax);
    }

    private double getRightOffsetMin() {
        double lo = Math.min(minPos, maxPos);
        double hi = Math.max(minPos, maxPos);
        if (invertRight) {
            return (2.0 * lo) - 1.0;
        }
        return -(hi - lo);
    }

    private double getRightOffsetMax() {
        double lo = Math.min(minPos, maxPos);
        double hi = Math.max(minPos, maxPos);
        if (invertRight) {
            return (2.0 * hi) - 1.0;
        }
        return (hi - lo);
    }

    private void writeServo(Servo servo, boolean enabled, double position) {
        if (servo == null) return;

        if (servo instanceof PwmControl) {
            if (enabled) {
                ((PwmControl) servo).setPwmEnable();
            } else {
                ((PwmControl) servo).setPwmDisable();
                return;
            }
        } else if (!enabled) {
            return;
        }

        servo.setPosition(position);
    }

    private void showTelemetry() {
        double leftCmd = computeLeftCmd();
        double rightCmd = computeRightCmd();
        double sharedMin = Math.max(minPos, invertRight ? (1.0 - maxPos) + rightOffset : minPos - rightOffset);
        double sharedMax = Math.min(maxPos, invertRight ? (1.0 - minPos) + rightOffset : maxPos - rightOffset);
        double encoderDeg = (spindexEnc != null) ? spindexEnc.getCurrentPosition() : Double.NaN;

        Logger.recordOutput("SpindexAligner/FloodgateAmps", floodgateCurrent);
        Logger.recordOutput("SpindexAligner/ModeRunning", modeRunning ? 1.0 : 0.0);
        Logger.recordOutput("SpindexAligner/ModeSelected", modeToNumber(selectedMode));
        Logger.recordOutput("SpindexAligner/LeftEnabled", leftEnabled ? 1.0 : 0.0);
        Logger.recordOutput("SpindexAligner/RightEnabled", rightEnabled ? 1.0 : 0.0);
        Logger.recordOutput("SpindexAligner/VerifyAutoSweep", verifyAutoSweep ? 1.0 : 0.0);
        Logger.recordOutput("SpindexAligner/VerifySweepDirection", verifySweepDirection);
        Logger.recordOutput("SpindexAligner/TargetPos", targetPos);
        Logger.recordOutput("SpindexAligner/LeftCmd", leftCmd);
        Logger.recordOutput("SpindexAligner/RightCmd", rightCmd);
        Logger.recordOutput("SpindexAligner/RightOffset", rightOffset);
        Logger.recordOutput("SpindexAligner/InvertRight", invertRight ? 1.0 : 0.0);
        Logger.recordOutput("SpindexAligner/SharedMin", sharedMin);
        Logger.recordOutput("SpindexAligner/SharedMax", sharedMax);
        Logger.recordOutput("SpindexAligner/LeftServoPos", spindexL.getPosition());
        Logger.recordOutput("SpindexAligner/RightServoPos", spindexR.getPosition());
        Logger.recordOutput("SpindexAligner/EncoderDeg", encoderDeg);
        Logger.recordOutput("SpindexAligner/CharacterizeServo", characterizeServo == CharacterizeServo.LEFT ? 0.0 : 1.0);
        Logger.recordOutput("SpindexAligner/LeftSampleCount", leftSampleCount);
        Logger.recordOutput("SpindexAligner/RightSampleCount", rightSampleCount);
        Logger.recordOutput("SpindexAligner/CompareMatchedCount", compareMatchedCount);
        Logger.recordOutput("SpindexAligner/CompareAvgSignedEncErr", compareAvgSignedEncErr);
        Logger.recordOutput("SpindexAligner/CompareAvgAbsEncErr", compareAvgAbsEncErr);
        Logger.recordOutput("SpindexAligner/CompareMaxAbsEncErr", compareMaxAbsEncErr);
        Logger.recordOutput("SpindexAligner/SuggestedRightOffsetDelta", suggestedRightOffsetDelta);

        Logger.recordOutput("FloodgateAmps", floodgateCurrent);

        telemetry.addData("Selected Mode", selectedMode);
        telemetry.addData("Mode Running", modeRunning);
        telemetry.addData("TargetPos", "%.4f", targetPos);
        telemetry.addData("LeftCmd", "%.4f", leftCmd);
        telemetry.addData("RightCmd", "%.4f", rightCmd);
        telemetry.addData("Left Enabled", leftEnabled);
        telemetry.addData("Right Enabled", rightEnabled);
        telemetry.addData("Invert Right", invertRight);
        telemetry.addData("Right Offset", "%.4f", rightOffset);
        telemetry.addData("Right Offset Range", "%.4f .. %.4f", getRightOffsetMin(), getRightOffsetMax());
        telemetry.addData("Floodgate Current (A)", "%.2f", floodgateCurrent);

        telemetry.addData("L getPosition", "%.4f", spindexL.getPosition());
        telemetry.addData("R getPosition", "%.4f", spindexR.getPosition());
        telemetry.addData("L PWM us", fmtPwmMicros(spindexL, leftCmd));
        telemetry.addData("R PWM us", fmtPwmMicros(spindexR, rightCmd));

        if (spindexEnc != null) {
            telemetry.addData("Spindex Enc Deg", "%.2f", spindexEnc.getCurrentPosition());
        } else {
            telemetry.addData("Spindex Enc Deg", "n/a");
        }

        telemetry.addLine("START: cycle mode (idle only), A: start mode, RS: stop mode");

        if (!modeRunning) {
            telemetry.addLine("Idle: pick mode with START, then press A to begin.");
        } else if (selectedMode == Mode.ALIGN) {
            telemetry.addLine("ALIGN: LS move target, Dpad/LB/RB fine+coarse, BACK center");
            telemetry.addLine("X toggle left, B toggle right, Y enable both");
        } else if (selectedMode == Mode.VERIFY) {
            telemetry.addData("AutoSweep", verifyAutoSweep);
            telemetry.addLine("VERIFY: both servos enabled and moving together.");
            telemetry.addLine("X toggle autosweep, Y reverse sweep direction, A center/stop sweep");
            telemetry.addLine("Dpad Up/Down: rightOffset +/- small, RT/LT: rightOffset +/- large");
            telemetry.addLine("If gears chatter, stop and adjust invertRight/offset/mechanical mesh.");
        } else {
            telemetry.addData("Characterize Servo", characterizeServo);
            telemetry.addData("L samples", leftSampleCount);
            telemetry.addData("R samples", rightSampleCount);
            telemetry.addData("Compare matched", compareMatchedCount);
            telemetry.addData("Avg signed err (deg)", fmt(compareAvgSignedEncErr));
            telemetry.addData("Avg abs err (deg)", fmt(compareAvgAbsEncErr));
            telemetry.addData("Max abs err (deg)", fmt(compareMaxAbsEncErr));
            telemetry.addData("L slope (deg/cmd)", fmt(leftDegPerCommand));
            telemetry.addData("R slope (deg/cmd)", fmt(rightDegPerCommand));
            telemetry.addData("Suggested rightOffset d", fmt(suggestedRightOffsetDelta));

            telemetry.addLine("CHAR: Y swap active servo (other PWM disabled)");
            telemetry.addLine("LS/Dpad/LB/RB move target, A capture point, B clear active set");
            telemetry.addLine("Dpad Up/Down: rightOffset +/- small, RT/LT: rightOffset +/- large");
            telemetry.addLine("X compare left-vs-right datasets, BACK center target");
        }

        telemetry.update();
    }

    private static boolean edge(boolean now, boolean prevVal) {
        return now && !prevVal;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(value, hi));
    }

    private static double wrapSignedDeg(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static String fmtPwmMicros(Servo servo, double commandedPos) {
        if (!(servo instanceof PwmControl)) return "n/a";

        PwmControl.PwmRange range = ((PwmControl) servo).getPwmRange();
        double pos = clamp(commandedPos, 0.0, 1.0);
        double pulseUs = range.usPulseLower + (pos * (range.usPulseUpper - range.usPulseLower));
        return String.format("%.1f us (range %.0f..%.0f)", pulseUs, range.usPulseLower, range.usPulseUpper);
    }

    private static void applyPwmRange(Servo servo) {
        if (!(servo instanceof PwmControl)) return;
        double lo = Math.min(pwmMinUs, pwmMaxUs);
        double hi = Math.max(pwmMinUs, pwmMaxUs);
        ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(lo, hi));
    }

    private static String fmt(double value) {
        if (Double.isNaN(value)) return "n/a";
        return String.format("%.3f", value);
    }

    private static double modeToNumber(Mode mode) {
        if (mode == Mode.ALIGN) return 0.0;
        if (mode == Mode.VERIFY) return 1.0;
        return 2.0;
    }
}
