package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Bench2: Motor Getters (bulk-backed)", group = "Bench")
public class Bench2_Getters_Bulk extends BenchBaseDriveOpMode {

    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;

    private final BenchUtil.RollingStats blockUs = new BenchUtil.RollingStats(200);

    public Bench2_Getters_Bulk() {
        // Bulk getters only make sense with MANUAL bulk cache so we control first-read timing.
        useManualBulkCaching = true;
    }

    @Override
    protected String getBenchName() {
        return "Bench2: Motor Getters (bulk-backed)";
    }

    @Override
    public void init() {
        super.init();
        lf = (DcMotorEx) leftFront;
        rf = (DcMotorEx) rightFront;
        lb = (DcMotorEx) leftBack;
        rb = (DcMotorEx) rightBack;

        telemetry.addLine("Measures 1x pos + 1x velocity per motor after clearBulkCache()");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Measure the first bulk-backed reads after cache clear.
        maybeClearBulkCache();

        long t0 = BenchUtil.nowNs();
        // CurrentPosition + Velocity are expected to be backed by Lynx bulk data.
        // Intentionally touch each motor once.
        lf.getCurrentPosition();
        lf.getVelocity();
        rf.getCurrentPosition();
        rf.getVelocity();
        lb.getCurrentPosition();
        lb.getVelocity();
        rb.getCurrentPosition();
        rb.getVelocity();
        long t1 = BenchUtil.nowNs();

        blockUs.add(BenchUtil.nsToUs(t1 - t0));

        driveFromGamepad();
        recordLoopDtMs();

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
}
