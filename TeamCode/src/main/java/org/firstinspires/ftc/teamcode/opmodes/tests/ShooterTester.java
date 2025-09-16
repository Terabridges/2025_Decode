package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

/**
 * Shooter Tester OpMode
 *
 * Uses your custom Robot + Subsystem framework.
 * Controls (during RUN):
 *  - RT: analog RPM (0..6000)
 *  - LB/RB: -100 / +100 RPM nudge
 *  - A/B: 3000 / 4500 RPM presets
 *  - X: stop
 *  - Y: reverse shooter direction (multiplier flip)
 *  - D-pad Up/Down: hood +0.01 / -0.01
 *
 * Pre-start (INIT-loop) options:
 *  - Left stick click: toggle encoder side (LEFT/RIGHT)
 */
@TeleOp(name = "ShooterTester", group = "Test")
public class ShooterTester extends LinearOpMode {

    // Match the clamp used in your Shooter subsystem
    private static final double MAX_RPM = 6000.0;

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

            // Presets
            if (gamepad1.a) bot.shooter.setRpm(3000);
            if (gamepad1.b) bot.shooter.setRpm(4500);

            // Nudges
            double lTrig = gamepad1.left_trigger; // 0..1
            if (lTrig > 0.02) {
                if (gamepad1.left_bumper) bot.shooter.nudgeRpm(-1000);
                if (gamepad1.right_bumper) bot.shooter.nudgeRpm(+1000);
            }
            else
            {
                if (gamepad1.left_bumper) bot.shooter.nudgeRpm(-100);
                if (gamepad1.right_bumper) bot.shooter.nudgeRpm(+100);
            }

            // Stop
            if (gamepad1.x) bot.shooter.stop();

            // Reverse direction (unjam/backspin)
            if (gamepad1.y) {
                bot.shooter.reverseDirection();
                sleep(150); // debounce
            }

            // Hood control
            if (gamepad1.dpad_up)    bot.shooter.nudgeHood(+0.01);
            if (gamepad1.dpad_down)  bot.shooter.nudgeHood(-0.01);

            // Update subsystems + telemetry (Shooter + Limelight)
            bot.update();
        }
    }
}
