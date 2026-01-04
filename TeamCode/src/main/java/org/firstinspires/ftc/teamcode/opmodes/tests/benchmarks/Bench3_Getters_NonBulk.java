package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Bench3: Motor Getters (non-bulk)", group = "Bench")
public class Bench3_Getters_NonBulk extends BenchBaseDriveOpMode {

    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;

    private final BenchUtil.RollingStats blockUs = new BenchUtil.RollingStats(200);

    public Bench3_Getters_NonBulk() {
        // Bulk caching doesn't help for non-bulk reads, but keep it on so loop structure matches Bench2.
        useManualBulkCaching = true;
    }

    @Override
    protected String getBenchName() {
        return "Bench3: Motor Getters (non-bulk)";
    }

    @Override
    public void init() {
        super.init();
        lf = (DcMotorEx) leftFront;
        rf = (DcMotorEx) rightFront;
        lb = (DcMotorEx) leftBack;
        rb = (DcMotorEx) rightBack;

        telemetry.addLine("Measures motor current reads (expected non-bulk, per-call Lynx cmd)");
        telemetry.update();
    }

    @Override
    public void loop() {
        maybeClearBulkCache();

        long t0 = BenchUtil.nowNs();
        // Motor current readings are typically NOT bulk-backed.
        lf.getCurrent(CurrentUnit.MILLIAMPS);
        rf.getCurrent(CurrentUnit.MILLIAMPS);
        lb.getCurrent(CurrentUnit.MILLIAMPS);
        rb.getCurrent(CurrentUnit.MILLIAMPS);
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
