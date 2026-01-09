package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.PsiKitLinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;

@TeleOp(name = "PsiKit Linear OpMode Minimal", group = "Test")
public class PsiKitLinearOpModeMinimal extends PsiKitLinearOpMode {

    @Override
    public int getRlogPort() {
        // If another device (e.g., Limelight) is already using port 5800/5801, this will change it
        // Update the port in AdvantageScope prefs.json to match.
        // If you are ok with AdvantageScope default of port 5800, you can omit this override.
        return 5802;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("PsiKit logging active (PsiKitLinearOpMode).\n");
        telemetry.addLine("Start this OpMode, move sticks, press buttons.");
        telemetry.addLine("Log file: /sdcard/FIRST/PsiKit/<OpMode>_log_<timestamp>.rlog");
        telemetry.addLine("Note: Use opModeIsActive()/opModeInInit() loops (not isStopRequested()).");
        telemetry.update();

        int loopCount = 0;

        while (opModeInInit()) {
            telemetry.addData("Status", "INIT");
            telemetry.addData("LoopCount", loopCount);
            telemetry.update();
            idle();
        }

        waitForStart();

        loopCount = 0;
        while (opModeIsActive()) {
            // USER CODE GOES HERE (read gamepads, update subsystems, etc.)

            Logger.recordOutput("Example/LoopCount", loopCount++);
            Logger.recordOutput("Example/Gamepad1LeftY", (double) gamepad1.left_stick_y);

            telemetry.addData("Status", "RUN");
            telemetry.addData("LoopCount", loopCount);
            telemetry.update();
        }
    }
}
