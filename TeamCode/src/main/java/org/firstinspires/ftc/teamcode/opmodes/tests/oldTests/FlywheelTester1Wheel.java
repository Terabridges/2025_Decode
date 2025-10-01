package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FlywheelTester1Wheel")
public class FlywheelTester1Wheel extends LinearOpMode {

    private DcMotorEx shooter;
    private static final double TICKS_PER_REV = 28.0;
    private double desiredRpm = 0.0;
    private boolean isRunning = false;
    private boolean fineMode = false;

    private boolean lastA = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD); // Change to REVERSE if needed
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle running state with A button
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                isRunning = !isRunning;
            }
            lastA = currentA;

            // Fine mode if left bumper is held
            fineMode = gamepad1.left_bumper;

            // RPM adjustments with dpad up/down
            double step = fineMode ? 10.0 : 100.0;

            boolean currentUp = gamepad1.dpad_up;
            if (currentUp && !lastDpadUp) {
                desiredRpm += step;
            }
            lastDpadUp = currentUp;

            boolean currentDown = gamepad1.dpad_down;
            if (currentDown && !lastDpadDown) {
                desiredRpm -= step;
            }
            lastDpadDown = currentDown;

            // Limit RPM
            if (desiredRpm < 0) desiredRpm = 0;
            if (desiredRpm > 6000) desiredRpm = 6000;

            // Set velocity
            double velocity = (desiredRpm * TICKS_PER_REV) / 60.0;
            if (!isRunning) {
                velocity = 0;
            }
            shooter.setVelocity(velocity);

            // Telemetry
            double measuredVel = shooter.getVelocity();
            double measuredRpm = (measuredVel * 60.0) / TICKS_PER_REV;

            telemetry.addData("Set RPM", desiredRpm);
            telemetry.addData("Measured RPM", measuredRpm);
            telemetry.addData("Running", isRunning ? "Yes" : "No");
            telemetry.addData("Adjustment Mode", fineMode ? "Fine (10 RPM)" : "Gross (100 RPM)");
            telemetry.update();
        }
    }
}