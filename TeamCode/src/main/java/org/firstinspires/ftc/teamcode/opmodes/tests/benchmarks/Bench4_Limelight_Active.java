package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bench4: Limelight (active polling)", group = "Bench")
public class Bench4_Limelight_Active extends BenchBaseDriveOpMode {

    private Limelight3A limelight;
    private LLResult latest;

    private final BenchUtil.RollingStats llUs = new BenchUtil.RollingStats(200);

    @Override
    protected String getBenchName() {
        return "Bench4: Limelight (active polling)";
    }

    @Override
    public void init() {
        super.init();
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Throwable t) {
            limelight = null;
        }

        telemetry.addData("Limelight present", limelight != null);
        telemetry.addLine("This starts Limelight + calls getLatestResult() each loop.");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        if (limelight != null) {
            try {
                limelight.setPollRateHz(100);
                limelight.start();
            } catch (Throwable ignored) {
            }
        }
    }

    @Override
    public void loop() {
        long t0 = BenchUtil.nowNs();
        if (limelight != null) {
            try {
                latest = limelight.getLatestResult();
            } catch (Throwable ignored) {
                latest = null;
            }
        }
        long t1 = BenchUtil.nowNs();
        llUs.add(BenchUtil.nsToUs(t1 - t0));

        driveFromGamepad();
        recordLoopDtMs();

        if (telemetryThrottle.shouldUpdate()) {
            telemetry.addLine(getBenchName());
            telemetry.addData("Limelight poll us (avg)", String.format("%.1f", llUs.mean()));
            telemetry.addData("Limelight poll us (p95)", String.format("%.1f", llUs.percentile(0.95)));
            telemetry.addData("Limelight poll us (max)", String.format("%.1f", llUs.max()));
            telemetry.addData("Has result", latest != null);
            telemetry.addData("Loop dt ms (avg)", String.format("%.3f", loopMs.mean()));
            telemetry.addData("Loop dt ms (p95)", String.format("%.3f", loopMs.percentile(0.95)));
            telemetry.addData("Loop dt ms (max)", String.format("%.3f", loopMs.max()));
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Throwable ignored) {
            }
        }
        super.stop();
    }
}
