package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

@TeleOp(name = "Spindex V3 Position Test", group = "Test")
public class SpindexV3PositionTest extends OpMode {

    private static final double SPINDEX_SCALE_STEP = 0.05;
    private static final double SPINDEX_SCALE_MIN = 0.10;
    private static final double SPINDEX_SCALE_MAX = 1.00;

    private Robot robot;
    private double spindexScale = 1.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

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
            spindexScale = Math.min(SPINDEX_SCALE_MAX, spindexScale + SPINDEX_SCALE_STEP);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            spindexScale = Math.max(SPINDEX_SCALE_MIN, spindexScale - SPINDEX_SCALE_STEP);
        }
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

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

        // Use existing subsystem movement path so both spindex actuators stay linked.
        double spindexPower = (gamepad1.right_trigger - gamepad1.left_trigger) * spindexScale;
        robot.intake.spindex.moveSpindexPow(spindexPower);

        robot.update();

        telemetry.addLine("Spindex V3 Position Test (Subsystem)");
        telemetry.addLine("Drive: g1 left stick + g1 right stick X");
        telemetry.addLine("Spindex power: g1 RT forward, g1 LT reverse");
        telemetry.addLine("Spindex speed scale: g1 dpad_up/down");
        telemetry.addData("Spindex Scale", "%.2f", spindexScale);
        telemetry.addData("Spindex Position (deg)", "%.2f", robot.intake.spindex.getPositionDeg());
        telemetry.addData("Spindex Analog (V)", "%.3f", robot.intake.spindex.getAnalogVoltage());
        telemetry.addData("Spindex Commanded Power", "%.2f", robot.intake.spindex.getCommandedPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.intake.spindex.moveSpindexPow(0.0);
        robot.other.drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
        robot.update();
    }
}
