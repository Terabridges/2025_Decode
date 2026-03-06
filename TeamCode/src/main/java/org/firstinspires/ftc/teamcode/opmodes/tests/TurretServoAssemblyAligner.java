package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;

@Configurable
@TeleOp(name = "TurretServoAssemblyAligner", group = "Test")
public class TurretServoAssemblyAligner extends OpMode {

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
    public static double leftOffset = 0.0;
    public static double rightOffset = 0.031;

    public static double verifySweepMin = 0.35;
    public static double verifySweepMax = 0.65;
    public static double verifySweepRatePerSec = 0.20;

    public static int maxCharacterizeSamples = 20;
    public static double compareCommandTolerance = 0.01;
    public static double pwmMinUs = 500.0;
    public static double pwmMaxUs = 2500.0;

    private Servo turretL;
    private Servo turretR;
    private AbsoluteAnalogEncoder turretEnc;

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
    private double leftNormalizedRangeRatio = Double.NaN;
    private double rightNormalizedRangeRatio = Double.NaN;

    private long lastLoopNs;

    @Override
    public void init() {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");

        applyPwmRange(turretL);
        applyPwmRange(turretR);

        try {
            AnalogInput analog = hardwareMap.get(AnalogInput.class, "turretAnalog");
            turretEnc = new AbsoluteAnalogEncoder(analog, 3.3, 0, 1);
        } catch (Exception ignored) {
            turretEnc = null;
        }

        lastLoopNs = System.nanoTime();
        targetPos = clamp(targetPos, minPos, maxPos);
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);

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
            targetPos = clamp(centerPos, minPos, maxPos);
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

        writeServo(turretL, leftEnabled, computeLeftCmd());
        writeServo(turretR, rightEnabled, computeRightCmd());

        showTelemetry();
    }

    @Override
    public void stop() {
        writeServo(turretL, false, computeLeftCmd());
        writeServo(turretR, false, computeRightCmd());
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

        targetPos = clamp(targetPos, minPos, maxPos);
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

        targetPos = clamp(targetPos, minPos, maxPos);
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
        targetPos = clamp(targetPos, minPos, maxPos);

        if (edge(current.a, previous.a)) {
            captureSample();
        }

        if (edge(current.b, previous.b)) {
            clearActiveSamples();
            compareMatchedCount = 0;
            compareAvgSignedEncErr = Double.NaN;
            compareAvgAbsEncErr = Double.NaN;
            compareMaxAbsEncErr = Double.NaN;
            suggestedRightOffsetDelta = Double.NaN;
            leftDegPerCommand = Double.NaN;
            rightDegPerCommand = Double.NaN;
            leftNormalizedRangeRatio = Double.NaN;
            rightNormalizedRangeRatio = Double.NaN;
        }

        if (edge(current.x, previous.x)) {
            computeCompareStats();
        }
    }

    private void captureSample() {
        if (turretEnc == null) return;

        int cap = clampSampleCap();
        double encoderDeg = turretEnc.getCurrentPosition();
        if (characterizeServo == CharacterizeServo.LEFT) {
            if (leftSampleCount >= cap) return;
            leftCmdSamples[leftSampleCount] = computeLeftCmd();
            leftEncSamples[leftSampleCount] = encoderDeg;
            leftSampleCount++;
        } else {
            if (rightSampleCount >= cap) return;
            rightCmdSamples[rightSampleCount] = computeRightCmd();
            rightEncSamples[rightSampleCount] = encoderDeg;
            rightSampleCount++;
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
        leftNormalizedRangeRatio = Double.NaN;
        rightNormalizedRangeRatio = Double.NaN;

        if (leftSampleCount == 0 || rightSampleCount == 0) return;

        double tolerance = Math.max(0.0, compareCommandTolerance);
        double sumSigned = 0.0;
        double sumAbs = 0.0;
        double maxAbs = 0.0;

        for (int li = 0; li < leftSampleCount; li++) {
            int bestRi = -1;
            double bestCmdDelta = Double.POSITIVE_INFINITY;

            for (int ri = 0; ri < rightSampleCount; ri++) {
                double cmdDelta = Math.abs(leftCmdSamples[li] - rightCmdSamples[ri]);
                if (cmdDelta <= tolerance && cmdDelta < bestCmdDelta) {
                    bestCmdDelta = cmdDelta;
                    bestRi = ri;
                }
            }

            if (bestRi >= 0) {
                double encErr = wrapSignedDeg(leftEncSamples[li] - rightEncSamples[bestRi]);
                double absErr = Math.abs(encErr);
                sumSigned += encErr;
                sumAbs += absErr;
                maxAbs = Math.max(maxAbs, absErr);
                compareMatchedCount++;
            }
        }

        if (compareMatchedCount > 0) {
            compareAvgSignedEncErr = sumSigned / compareMatchedCount;
            compareAvgAbsEncErr = sumAbs / compareMatchedCount;
            compareMaxAbsEncErr = maxAbs;

            leftDegPerCommand = estimateSlopeDegPerPos(leftCmdSamples, leftEncSamples, leftSampleCount);
            rightDegPerCommand = estimateSlopeDegPerPos(rightCmdSamples, rightEncSamples, rightSampleCount);

            if (!Double.isNaN(leftDegPerCommand)) {
                leftNormalizedRangeRatio = leftDegPerCommand / 360.0;
            }
            if (!Double.isNaN(rightDegPerCommand)) {
                rightNormalizedRangeRatio = rightDegPerCommand / 360.0;
            }

            if (!Double.isNaN(rightDegPerCommand) && Math.abs(rightDegPerCommand) > 1e-6) {
                suggestedRightOffsetDelta = compareAvgSignedEncErr / rightDegPerCommand;
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
            double dy = encSamples[i] - meanY;
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
        return clamp(targetPos + leftOffset, minPos, maxPos);
    }

    private double computeRightCmd() {
        double base = invertRight ? (1.0 - targetPos) : targetPos;
        return clamp(base + rightOffset, minPos, maxPos);
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
        telemetry.addData("Selected Mode", selectedMode);
        telemetry.addData("Mode Running", modeRunning);
        telemetry.addData("TargetPos", "%.4f", targetPos);
        telemetry.addData("LeftCmd", "%.4f", computeLeftCmd());
        telemetry.addData("RightCmd", "%.4f", computeRightCmd());
        telemetry.addData("Left Enabled", leftEnabled);
        telemetry.addData("Right Enabled", rightEnabled);
        telemetry.addData("Invert Right", invertRight);
        telemetry.addData("Left Offset", "%.4f", leftOffset);
        telemetry.addData("Right Offset", "%.4f", rightOffset);

        telemetry.addData("L getPosition", "%.4f", turretL.getPosition());
        telemetry.addData("R getPosition", "%.4f", turretR.getPosition());
        telemetry.addData("L PWM us", fmtPwmMicros(turretL, computeLeftCmd()));
        telemetry.addData("R PWM us", fmtPwmMicros(turretR, computeRightCmd()));

        if (turretEnc != null) {
            telemetry.addData("Turret Enc Deg", "%.2f", turretEnc.getCurrentPosition());
        } else {
            telemetry.addData("Turret Enc Deg", "not available");
        }

        telemetry.addLine("START: cycle mode (idle only), A: start mode, RS: stop mode");

        if (!modeRunning) {
            telemetry.addLine("IDLE: outputs disabled so turret will not move.");
            telemetry.addLine("Select mode with START, then press A to run selected mode.");
        } else if (selectedMode == Mode.ALIGN) {
            telemetry.addLine("ALIGN: Set both servos to same target before gear reassembly.");
            telemetry.addLine("LS Y=continuous, Dpad L/R=small, LB/RB=large, A=center");
            telemetry.addLine("X toggle left PWM, B toggle right PWM, Y enable both");
        } else if (selectedMode == Mode.VERIFY) {
            telemetry.addData("AutoSweep", verifyAutoSweep);
            telemetry.addLine("VERIFY: both servos enabled and moving together.");
            telemetry.addLine("X toggle autosweep, Y reverse sweep direction, A center/stop sweep");
            telemetry.addLine("If gears chatter, stop and adjust invertRight/offsets/mechanical mesh.");
        } else {
            telemetry.addData("Characterize Servo", characterizeServo);
            telemetry.addData("Left Samples", leftSampleCount);
            telemetry.addData("Right Samples", rightSampleCount);
            telemetry.addData("Command Tol", "%.4f", compareCommandTolerance);
            telemetry.addData("Matched Points", compareMatchedCount);
            telemetry.addData("Avg Enc Delta Signed (L-R)", fmt(compareAvgSignedEncErr));
            telemetry.addData("Avg |Enc Delta| Deg", fmt(compareAvgAbsEncErr));
            telemetry.addData("Max |Enc Delta| Deg", fmt(compareMaxAbsEncErr));
            telemetry.addData("Suggested rightOffset Δ", fmt(suggestedRightOffsetDelta));
            if (!Double.isNaN(suggestedRightOffsetDelta)) {
                telemetry.addData("Suggested rightOffset", fmt(rightOffset + suggestedRightOffsetDelta));
            }
            telemetry.addData("Left deg per cmd", fmt(leftDegPerCommand));
            telemetry.addData("Right deg per cmd", fmt(rightDegPerCommand));
            telemetry.addData("Left ratio (deg/360)", fmt(leftNormalizedRangeRatio));
            telemetry.addData("Right ratio (deg/360)", fmt(rightNormalizedRangeRatio));

            if (leftSampleCount > 0) {
                telemetry.addData("Last Left", "cmd %.4f -> enc %.2f",
                        leftCmdSamples[leftSampleCount - 1], leftEncSamples[leftSampleCount - 1]);
            }
            if (rightSampleCount > 0) {
                telemetry.addData("Last Right", "cmd %.4f -> enc %.2f",
                        rightCmdSamples[rightSampleCount - 1], rightEncSamples[rightSampleCount - 1]);
            }

            telemetry.addLine("CHAR: Y swap active servo (other PWM disabled)");
            telemetry.addLine("LS/Dpad/LB/RB move target, A capture point, B clear active set");
            telemetry.addLine("X compare left-vs-right datasets, BACK center target");
        }

        telemetry.update();
    }

    private static boolean edge(boolean now, boolean prevVal) {
        return now && !prevVal;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(Math.min(value, Math.max(lo, hi)), Math.min(lo, hi));
    }

    private static double wrapSignedDeg(double degrees) {
        double wrapped = ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return wrapped;
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
}
