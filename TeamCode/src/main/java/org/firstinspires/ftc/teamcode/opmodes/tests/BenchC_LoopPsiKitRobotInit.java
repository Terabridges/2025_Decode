package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "BenchC Loop PsiKit + RobotInit", group = "Benchmark")
public class BenchC_LoopPsiKitRobotInit extends QuickLoopBenchBase {

    private Robot robot;
    private boolean previousColorBackgroundPolling;

    @Override
    protected void onBenchInit() {
        previousColorBackgroundPolling = FtcLogTuning.processColorDistanceSensorsInBackground;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void stop() {
        FtcLogTuning.processColorDistanceSensorsInBackground = previousColorBackgroundPolling;
    }

    @Override
    protected boolean isPsiKitLoopLoggingEnabled() {
        return true;
    }

    @Override
    protected String benchName() {
        return "BenchC_PsiKit_RobotInitOnly";
    }
}
