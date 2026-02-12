package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

@TeleOp(name = "Turret V3 Test", group = "Test")
public class TurretV3Test extends OpMode {

    private static final double TURRET_SCALE_STEP = 0.05;
    private static final double TURRET_SCALE_MIN = 0.10;
    private static final double TURRET_SCALE_MAX = 1.00;

    private Robot robot;
    private double turretScale = 0.50;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private TurretMode turretMode = TurretMode.BOTH;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevB = false;

    private enum TurretMode {
        BOTH,
        LEFT_ONLY,
        RIGHT_ONLY
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void start() {
        robot.toInit();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !prevDpadUp) {
            turretScale = Math.min(TURRET_SCALE_MAX, turretScale + TURRET_SCALE_STEP);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            turretScale = Math.max(TURRET_SCALE_MIN, turretScale - TURRET_SCALE_STEP);
        }
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        if (gamepad1.x && !prevX) {
            turretMode = TurretMode.BOTH;
        }
        if (gamepad1.y && !prevY) {
            turretMode = TurretMode.LEFT_ONLY;
        }
        if (gamepad1.b && !prevB) {
            turretMode = TurretMode.RIGHT_ONLY;
        }
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevB = gamepad1.b;

        // Robot-centric mecanum drive
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        robot.other.drive.setDrivePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        double turretInput = gamepad1.right_trigger - gamepad1.left_trigger;
        double turretPower = turretInput * turretScale;
        switch (turretMode) {
            case BOTH:
                robot.outtake.turret.moveTurretPows(turretPower, turretPower);
                break;
            case LEFT_ONLY:
                robot.outtake.turret.moveTurretPows(turretPower, 0.0);
                break;
            case RIGHT_ONLY:
                robot.outtake.turret.moveTurretPows(0.0, turretPower);
                break;
        }

        robot.update();

        telemetry.addLine("Turret V3 Test (Subsystem)");
        telemetry.addLine("Drive: g1 left stick + g1 right stick X");
        telemetry.addLine("Turret: g1 RT forward, g1 LT reverse");
        telemetry.addLine("Turret speed scale: g1 dpad_up/down");
        telemetry.addLine("Turret mode: g1 X=BOTH, Y=LEFT, B=RIGHT");
        telemetry.addData("Turret Mode", turretMode);
        telemetry.addData("Turret Scale", "%.2f", turretScale);
        telemetry.addData("Turret Input", "%.2f", turretInput);
        telemetry.addData("Turret Commanded Power", "%.2f", robot.outtake.turret.getCommandedPower());
        telemetry.addData("Turret Left Power", "%.2f", robot.outtake.turret.getCommandedLeftPower());
        telemetry.addData("Turret Right Power", "%.2f", robot.outtake.turret.getCommandedRightPower());
        telemetry.addData("Turret Position (deg)", "%.2f", robot.outtake.turret.getPositionDeg());
        telemetry.addData("Turret Analog (V)", "%.3f", robot.outtake.turret.getAnalogVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.outtake.turret.moveTurretPow(0.0);
        robot.other.drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
        robot.update();
    }
}
