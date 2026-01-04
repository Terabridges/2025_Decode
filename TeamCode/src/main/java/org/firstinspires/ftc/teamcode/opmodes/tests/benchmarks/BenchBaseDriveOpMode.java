package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

/**
 * Shared drivetrain-only logic for benchmark OpModes.
 *
 * Intentionally does NOT construct Robot/Drive/Vision/etc. to avoid pulling extra devices.
 */
abstract class BenchBaseDriveOpMode extends OpMode {

    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftBack;
    protected DcMotor rightBack;

    protected List<LynxModule> hubs;

    protected long lastLoopEndNs = Long.MIN_VALUE;
    protected final BenchUtil.RollingStats loopMs = new BenchUtil.RollingStats(200);
    protected final BenchUtil.TelemetryThrottle telemetryThrottle = new BenchUtil.TelemetryThrottle(0.25);

    /** If true, set hubs to MANUAL and clear bulk cache each loop. */
    protected boolean useManualBulkCaching = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addLine(getBenchName());
        telemetry.addLine("Drive-only: LF/RF/LB/RB");
        telemetry.addData("Manual bulk caching", useManualBulkCaching);
        telemetry.update();
    }

    @Override
    public void start() {
        lastLoopEndNs = BenchUtil.nowNs();
        loopMs.add(0.0);

        if (useManualBulkCaching && hubs != null) {
            for (LynxModule hub : hubs) {
                try {
                    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                } catch (Throwable ignored) {
                }
            }
        }
    }

    protected abstract String getBenchName();

    protected void maybeClearBulkCache() {
        if (!useManualBulkCaching || hubs == null) return;
        for (LynxModule hub : hubs) {
            try {
                hub.clearBulkCache();
            } catch (Throwable ignored) {
            }
        }
    }

    protected void driveFromGamepad() {
        // Standard mecanum mixing.
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        double lf = (y + x + rx) / denominator;
        double rf = (y - x - rx) / denominator;
        double lb = (y - x + rx) / denominator;
        double rb = (y + x - rx) / denominator;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    protected double recordLoopDtMs() {
        long now = BenchUtil.nowNs();
        double dtMs = 0.0;
        if (lastLoopEndNs != Long.MIN_VALUE) {
            dtMs = BenchUtil.nsToMs(now - lastLoopEndNs);
        }
        lastLoopEndNs = now;
        loopMs.add(dtMs);
        return dtMs;
    }

    protected void telemetryLoopStats() {
        if (!telemetryThrottle.shouldUpdate()) return;

        telemetry.addLine(getBenchName());
        telemetry.addData("Loop dt ms (avg)", String.format("%.3f", loopMs.mean()));
        telemetry.addData("Loop dt ms (p95)", String.format("%.3f", loopMs.percentile(0.95)));
        telemetry.addData("Loop dt ms (max)", String.format("%.3f", loopMs.max()));
        telemetry.addData("Samples", loopMs.size());
        telemetry.update();
    }
}
