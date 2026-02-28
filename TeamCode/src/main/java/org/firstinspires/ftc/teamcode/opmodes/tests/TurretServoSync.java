package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;

@Configurable
@TeleOp(name = "TurretServoSync", group = "Test")
public class TurretServoSync extends OpMode {

    public static double targetPower = 0.00;
    public static double maxPower = 1.00;
    public static double stickRatePerSec = 0.50;
    public static double smallStep = 0.01;
    public static double largeStep = 0.05;
    public static boolean useExactSetpoint = false;
    public static double exactSetpoint = 0.00;

    public static boolean enableLeft = true;
    public static boolean enableRight = true;
    public static boolean invertRight = false;
    public static double leftOffset = 0.0;
    public static double rightOffset = 0.0;

    private CRServo turretL;
    private CRServo turretR;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    private long lastLoopNs;

    @Override
    public void init() {
        turretL = hardwareMap.get(CRServo.class, "turretL");
        turretR = hardwareMap.get(CRServo.class, "turretR");
        lastLoopNs = System.nanoTime();
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);

        long nowNs = System.nanoTime();
        double dtSec = Math.max(0.0, (nowNs - lastLoopNs) / 1e9);
        lastLoopNs = nowNs;

        if (useExactSetpoint) {
            targetPower = exactSetpoint;
        } else {
            targetPower += (-current.left_stick_y) * stickRatePerSec * dtSec;

            if (edge(current.dpad_right, previous.dpad_right)) {
                targetPower += smallStep;
            }
            if (edge(current.dpad_left, previous.dpad_left)) {
                targetPower -= smallStep;
            }
            if (edge(current.right_bumper, previous.right_bumper)) {
                targetPower += largeStep;
            }
            if (edge(current.left_bumper, previous.left_bumper)) {
                targetPower -= largeStep;
            }
            if (edge(current.a, previous.a)) {
                targetPower = 0.00;
            }
            if (edge(current.x, previous.x)) {
                targetPower = -maxPower;
            }
            if (edge(current.b, previous.b)) {
                targetPower = maxPower;
            }
        }

        targetPower = clamp(targetPower, -maxPower, maxPower);

        double leftCmd = clamp(targetPower + leftOffset, -maxPower, maxPower);
        double rightBase = invertRight ? -targetPower : targetPower;
        double rightCmd = clamp(rightBase + rightOffset, -maxPower, maxPower);

        if (enableLeft) {
            turretL.setPower(leftCmd);
        } else {
            turretL.setPower(0.0);
        }
        if (enableRight) {
            turretR.setPower(rightCmd);
        } else {
            turretR.setPower(0.0);
        }

        telemetry.addData("Target Power", "%.4f", targetPower);
        telemetry.addData("ExactMode", useExactSetpoint);
        telemetry.addData("ExactSetpoint", "%.4f", exactSetpoint);
        telemetry.addData("LeftCmd", "%.4f", leftCmd);
        telemetry.addData("RightCmd", "%.4f", rightCmd);
        telemetry.addData("LeftEnabled", enableLeft);
        telemetry.addData("RightEnabled", enableRight);
        telemetry.addData("InvertRight", invertRight);
        telemetry.addData("LeftOffset", "%.4f", leftOffset);
        telemetry.addData("RightOffset", "%.4f", rightOffset);
        telemetry.addLine("Controls: LS Y=continuous, Dpad L/R=small, LB/RB=large");
        telemetry.addLine("A=zero, X=-max, B=+max");
        telemetry.update();
    }

    private static boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(Math.min(value, Math.max(lo, hi)), Math.min(lo, hi));
    }
}
