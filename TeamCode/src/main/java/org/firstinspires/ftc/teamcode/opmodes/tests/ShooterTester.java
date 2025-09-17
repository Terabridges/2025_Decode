package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

@TeleOp(name = "ShooterTester", group = "Test")
public class ShooterTester extends LinearOpMode {

    // Match the clamp used in your Shooter subsystem
    private static final double MAX_RPM = 6000.0;

    private boolean fineMode = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void runOpMode() {
        Robot bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        bot.toInit();

        waitForStart();

        while (opModeIsActive()) {
            // --- RPM control ---
            double rTrig = gamepad1.right_trigger; // 0..1
            if (rTrig > 0.02) {
                bot.shooter.setRpm(rTrig * MAX_RPM);
            }

            fineMode = gamepad1.left_bumper;

            // RPM adjustments with dpad up/down
            double stepRPM = fineMode ? 1000.0 : 100.0;

            double stepHood = fineMode ? 0.1 : 0.01;

            // Presets
            if (gamepad1.a) bot.shooter.setRpm(2500);
            if (gamepad1.b) bot.shooter.setRpm(4000);

            // Nudges
            boolean currentUp = gamepad1.dpad_up;
            if (currentUp && !lastDpadUp) {
                if (gamepad1.dpad_up) bot.shooter.nudgeRpm(stepRPM);
            }
            lastDpadUp = currentUp;

            boolean currentDown = gamepad1.dpad_down;
            if (currentDown && !lastDpadDown) {
                if (gamepad1.dpad_down) bot.shooter.nudgeRpm(-stepRPM);
            }
            lastDpadDown = currentDown;


            // Stop
            if (gamepad1.x) bot.shooter.stop();

            // Reverse direction (unjam/backspin)
            if (gamepad1.y) {
                bot.shooter.reverseDirection();
                sleep(150); // debounce
            }

            //Hood control

//            boolean currentRight = gamepad1.dpad_right;
//            if (currentRight && !lastDpadRight) {
//                if (gamepad1.dpad_right) bot.shooter.nudgeHood(stepHood);
//            }
//            lastDpadRight = currentRight;

//            boolean currentLeft = gamepad1.dpad_left;
//            if (currentLeft && !lastDpadLeft) {
//                if (gamepad1.dpad_left) bot.shooter.nudgeHood(-stepHood);
//            }
//            lastDpadLeft = currentLeft;

            // Update subsystems + telemetry (Shooter + Limelight)
            bot.update();
        }
    }
}
