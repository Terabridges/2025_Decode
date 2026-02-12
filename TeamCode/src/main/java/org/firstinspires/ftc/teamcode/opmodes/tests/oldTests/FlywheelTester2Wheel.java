package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
//@Configurable
@TeleOp(name = "FlywheelTester2Wheel")
public class FlywheelTester2Wheel extends LinearOpMode {

    private DcMotorEx shooterL;
    private DcMotorEx shooterR;
    private static final double TICKS_PER_REV = 28.0;
    public static double desiredRpm = 0.0;
    public static double measuredRpm = 0.0;
    private boolean isRunning = false;
    private boolean fineMode = false;

    private boolean lastA = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    public static double invert = 1;

    @Override
    public void runOpMode() {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.REVERSE); // Change to REVERSE if needed
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterR.setDirection(DcMotorSimple.Direction.FORWARD); // Change to REVERSE if needed
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            shooterL.setVelocity(velocity*invert);
            shooterR.setVelocity(velocity*invert);

            // Telemetry
            double measuredVel = shooterL.getVelocity();
            measuredRpm = (measuredVel * 60.0) / TICKS_PER_REV;

            telemetry.addData("Set RPM", desiredRpm);
            telemetry.addData("Measured RPM", measuredRpm*invert);
            telemetry.addData("Running", isRunning ? "Yes" : "No");
            telemetry.addData("Adjustment Mode", fineMode ? "Fine (10 RPM)" : "Gross (100 RPM)");
            telemetry.update();
        }
    }
}