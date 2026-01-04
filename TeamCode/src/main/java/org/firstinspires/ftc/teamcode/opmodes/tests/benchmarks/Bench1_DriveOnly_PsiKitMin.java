package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.FtcLogTuning;

@TeleOp(name = "Bench1: DriveOnly + PsiKitMin", group = "Bench")
public class Bench1_DriveOnly_PsiKitMin extends BenchBaseDriveOpMode {

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    @Override
    protected String getBenchName() {
        return "Bench1: DriveOnly + PsiKitMin";
    }

    @Override
    public void init() {
        super.init();

        // Keep this bench focused: disable optional high-cost background logging.
        psiKit.enablePinpointOdometryLogging = false;

        // Ensure non-bulk throttles are off so this measures PsiKit overhead itself.
        FtcLogTuning.nonBulkReadPeriodSec = 0.0;
        FtcLogTuning.prefetchBulkDataEachLoop = false;
        FtcLogTuning.pinpointReadPeriodSec = 0.0;

        // Start PsiKit in init so it wraps hardwareMap before start.
        psiKit.start(this, 5802);

        telemetry.addLine("PsiKit started (port 5802)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double beforeUserStart = Logger.getRealTimestamp();
        Logger.periodicBeforeUser();
        psiKit.logOncePerLoop(this);
        double beforeUserEnd = Logger.getRealTimestamp();

        driveFromGamepad();

        double loopDtMs = recordLoopDtMs();
        Logger.recordOutput("Bench/LoopDtMs", loopDtMs);
        Logger.recordOutput("Bench/PsiKitBeforeUserMs", (beforeUserEnd - beforeUserStart) * 1000.0);

        double afterUserStart = Logger.getRealTimestamp();
        Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
        );

        telemetryLoopStats();
    }

    @Override
    public void stop() {
        try {
            psiKit.end();
        } finally {
            super.stop();
        }
    }
}
