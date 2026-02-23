package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "BenchD Loop PsiKit + RobotInit (BulkOnly)", group = "Benchmark")
public class BenchD_LoopPsiKitRobotInitBulkOnly extends QuickLoopBenchBase {

    private Robot robot;
    private boolean previousBulkOnly;
    private boolean previousPrefetchBulk;
    private boolean previousColorBackgroundPolling;

    @Override
    protected void onBenchInit() {
        previousBulkOnly = FtcLogTuning.bulkOnlyLogging;
        previousPrefetchBulk = FtcLogTuning.prefetchBulkDataEachLoop;
        previousColorBackgroundPolling = FtcLogTuning.processColorDistanceSensorsInBackground;

        FtcLogTuning.bulkOnlyLogging = true;
        FtcLogTuning.prefetchBulkDataEachLoop = true;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void stop() {
        FtcLogTuning.bulkOnlyLogging = previousBulkOnly;
        FtcLogTuning.prefetchBulkDataEachLoop = previousPrefetchBulk;
        FtcLogTuning.processColorDistanceSensorsInBackground = previousColorBackgroundPolling;
    }

    @Override
    protected boolean isPsiKitLoopLoggingEnabled() {
        return true;
    }

    @Override
    protected String benchName() {
        return "BenchD_PsiKit_RobotInit_BulkOnly";
    }
}
