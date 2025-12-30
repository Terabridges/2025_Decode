package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

@TeleOp(name = "PsiKit Iterative Base Minimal", group = "Test")
public class PsiKitOpModeMinimal extends PsiKitIterativeOpMode {

    private static final int RLOG_PORT = 5802;
    private int loopCount = 0;

    @Override
    protected int getRlogPort() {
        return RLOG_PORT;
    }

    @Override
    protected void onPsiKitInit() {
        telemetry.addLine("PsiKit logging active (OpMode).");
        telemetry.addLine("Log file: /sdcard/FIRST/PsiKit/<OpMode>_log_<timestamp>.rlog");
        telemetry.update();
    }

    @Override
    protected void onPsiKitInitLoop() {
        telemetry.addData("Status", "INIT");
        telemetry.addData("LoopCount", loopCount);
        telemetry.update();
    }

    @Override
    protected void onPsiKitStart() {
        loopCount = 0;
    }

    @Override
    protected void onPsiKitLoop() {
        // USER CODE GOES HERE (read gamepads, update subsystems, etc.)
        Logger.recordOutput("Example/LoopCount", loopCount++);
        Logger.recordOutput("Example/Gamepad1LeftY", (double) gamepad1.left_stick_y);

        telemetry.addData("Status", "RUN");
        telemetry.addData("LoopCount", loopCount);
        telemetry.update();
    }
}
