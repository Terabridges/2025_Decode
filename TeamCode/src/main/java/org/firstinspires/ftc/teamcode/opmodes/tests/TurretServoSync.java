package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "TurretServoSync", group = "Test")
public class TurretServoSync extends OpMode {

    public static double targetPos = 0.50;
    public static double minPos = 0.00;
    public static double maxPos = 1.00;
    public static double stickRatePerSec = 0.35;
    public static double smallStep = 0.0025;
    public static double largeStep = 0.01;
    public static boolean useExactSetpoint = false;
    public static double exactSetpoint = 0.50;

    public static boolean enableLeft = true;
    public static boolean enableRight = true;
    public static boolean invertRight = false;
    public static double leftOffset = 0.0;
    public static double rightOffset = 0.0;

    private Servo turretL;
    private Servo turretR;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    private long lastLoopNs;

    @Override
    public void init() {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");
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
            targetPos = exactSetpoint;
        } else {
            targetPos += (-current.left_stick_y) * stickRatePerSec * dtSec;

            if (edge(current.dpad_right, previous.dpad_right)) {
                targetPos += smallStep;
            }
            if (edge(current.dpad_left, previous.dpad_left)) {
                targetPos -= smallStep;
            }
            if (edge(current.right_bumper, previous.right_bumper)) {
                targetPos += largeStep;
            }
            if (edge(current.left_bumper, previous.left_bumper)) {
                targetPos -= largeStep;
            }
            if (edge(current.a, previous.a)) {
                targetPos = 0.50;
            }
            if (edge(current.x, previous.x)) {
                targetPos = minPos;
            }
            if (edge(current.b, previous.b)) {
                targetPos = maxPos;
            }
        }

        targetPos = clamp(targetPos, minPos, maxPos);

        double leftCmd = clamp(targetPos + leftOffset, minPos, maxPos);
        double rightBase = invertRight ? (1.0 - targetPos) : targetPos;
        double rightCmd = clamp(rightBase + rightOffset, minPos, maxPos);

        if (enableLeft) {
            turretL.setPosition(leftCmd);
        }
        if (enableRight) {
            turretR.setPosition(rightCmd);
        }

        telemetry.addData("Target", "%.4f", targetPos);
        telemetry.addData("ExactMode", useExactSetpoint);
        telemetry.addData("ExactSetpoint", "%.4f", exactSetpoint);
        telemetry.addData("LeftCmd", "%.4f", leftCmd);
        telemetry.addData("RightCmd", "%.4f", rightCmd);
        telemetry.addData("LeftActual", "%.4f", turretL.getPosition());
        telemetry.addData("RightActual", "%.4f", turretR.getPosition());
        telemetry.addData("LeftEnabled", enableLeft);
        telemetry.addData("RightEnabled", enableRight);
        telemetry.addData("InvertRight", invertRight);
        telemetry.addData("LeftOffset", "%.4f", leftOffset);
        telemetry.addData("RightOffset", "%.4f", rightOffset);
        telemetry.addLine("Controls: LS Y=continuous, Dpad L/R=small, LB/RB=large");
        telemetry.addLine("A=center, X=min, B=max");
        telemetry.update();
    }

    private static boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(Math.min(value, Math.max(lo, hi)), Math.min(lo, hi));
    }
}
