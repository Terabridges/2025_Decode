package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.FtcLoggingSession;

@TeleOp(name = "Bench3b: Getters (non-bulk) + PsiKitMin", group = "Bench")
public class Bench3b_Getters_NonBulk_PsiKitMin extends BenchBaseDriveOpMode {

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;

    private final BenchUtil.RollingStats blockUs = new BenchUtil.RollingStats(200);

    public Bench3b_Getters_NonBulk_PsiKitMin() {
        // Keep structure similar to Bench2b; bulk caching doesn't help for these reads.
        useManualBulkCaching = true;
    }

    @Override
    protected String getBenchName() {
        return "Bench3b: Getters (non-bulk) + PsiKitMin";
    }

    @Override
    public void init() {
        super.init();
        lf = (DcMotorEx) leftFront;
        rf = (DcMotorEx) rightFront;
        lb = (DcMotorEx) leftBack;
        rb = (DcMotorEx) rightBack;

        psiKit.enablePinpointOdometryLogging = false;

        FtcLogTuning.nonBulkReadPeriodSec = 0.0;
        FtcLogTuning.prefetchBulkDataEachLoop = false;
        FtcLogTuning.pinpointReadPeriodSec = 0.0;

        psiKit.start(this, 5802);

        telemetry.addLine("Measures motor current reads (expected non-bulk, per-call Lynx cmd) + writes .rlog");
        telemetry.addLine("PsiKit started (port 5802)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double beforeUserStart = Logger.getRealTimestamp();
        Logger.periodicBeforeUser();
        psiKit.logOncePerLoop(this);
        double beforeUserEnd = Logger.getRealTimestamp();

        maybeClearBulkCache();

        long t0 = BenchUtil.nowNs();
        lf.getCurrent(CurrentUnit.MILLIAMPS);
        rf.getCurrent(CurrentUnit.MILLIAMPS);
        lb.getCurrent(CurrentUnit.MILLIAMPS);
        rb.getCurrent(CurrentUnit.MILLIAMPS);
        long t1 = BenchUtil.nowNs();

        double thisBlockUs = BenchUtil.nsToUs(t1 - t0);
        blockUs.add(thisBlockUs);

        driveFromGamepad();

        double loopDtMs = recordLoopDtMs();
        Logger.recordOutput("Bench/LoopDtMs", loopDtMs);
        Logger.recordOutput("Bench/GetterBlockUs", thisBlockUs);
        Logger.recordOutput("Bench/PsiKitBeforeUserMs", (beforeUserEnd - beforeUserStart) * 1000.0);

        double afterUserStart = Logger.getRealTimestamp();
        Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
        );

        if (telemetryThrottle.shouldUpdate()) {
            telemetry.addLine(getBenchName());
            telemetry.addData("Getter block us (avg)", String.format("%.1f", blockUs.mean()));
            telemetry.addData("Getter block us (p95)", String.format("%.1f", blockUs.percentile(0.95)));
            telemetry.addData("Getter block us (max)", String.format("%.1f", blockUs.max()));
            telemetry.addData("Loop dt ms (avg)", String.format("%.3f", loopMs.mean()));
            telemetry.addData("Loop dt ms (p95)", String.format("%.3f", loopMs.percentile(0.95)));
            telemetry.addData("Loop dt ms (max)", String.format("%.3f", loopMs.max()));
            telemetry.update();
        }
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
