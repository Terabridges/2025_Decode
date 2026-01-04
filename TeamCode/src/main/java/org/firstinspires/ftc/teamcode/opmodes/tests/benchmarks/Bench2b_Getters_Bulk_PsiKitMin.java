package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.FtcLoggingSession;

@TeleOp(name = "Bench2b: Getters (bulk) + PsiKitMin", group = "Bench")
public class Bench2b_Getters_Bulk_PsiKitMin extends BenchBaseDriveOpMode {

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;

    private final BenchUtil.RollingStats blockUs = new BenchUtil.RollingStats(200);

    public Bench2b_Getters_Bulk_PsiKitMin() {
        // Bulk getters only make sense with MANUAL bulk cache so we control first-read timing.
        useManualBulkCaching = true;
    }

    @Override
    protected String getBenchName() {
        return "Bench2b: Getters (bulk) + PsiKitMin";
    }

    @Override
    public void init() {
        super.init();
        lf = (DcMotorEx) leftFront;
        rf = (DcMotorEx) rightFront;
        lb = (DcMotorEx) leftBack;
        rb = (DcMotorEx) rightBack;

        // Keep this bench focused: disable optional high-cost background logging.
        psiKit.enablePinpointOdometryLogging = false;

        // Avoid adding extra throttling/prefetch behavior that could hide the cost of the getter block.
        FtcLogTuning.nonBulkReadPeriodSec = 0.0;
        FtcLogTuning.prefetchBulkDataEachLoop = false;
        FtcLogTuning.pinpointReadPeriodSec = 0.0;

        psiKit.start(this, 5802);

        telemetry.addLine("Measures 1x pos + 1x velocity per motor after clearBulkCache() + writes .rlog");
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
        lf.getCurrentPosition();
        lf.getVelocity();
        rf.getCurrentPosition();
        rf.getVelocity();
        lb.getCurrentPosition();
        lb.getVelocity();
        rb.getCurrentPosition();
        rb.getVelocity();
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
