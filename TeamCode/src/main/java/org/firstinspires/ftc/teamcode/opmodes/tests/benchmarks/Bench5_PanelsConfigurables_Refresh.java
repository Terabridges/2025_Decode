package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bench5: PanelsConfigurables refresh", group = "Bench")
public class Bench5_PanelsConfigurables_Refresh extends BenchBaseDriveOpMode {

    @Configurable
    public static final class BenchPanelsConfig {
        public static double testValue = 1.0;
    }

    @Override
    protected String getBenchName() {
        return "Bench5: PanelsConfigurables refresh";
    }

    @Override
    public void init() {
        super.init();

        long t0 = BenchUtil.nowNs();
        try {
            Object instance = null;
            try {
                instance = PanelsConfigurables.class.getMethod("getInstance").invoke(null);
            } catch (Throwable ignored) {
            }
            if (instance == null) {
                try {
                    instance = PanelsConfigurables.class.getMethod("instance").invoke(null);
                } catch (Throwable ignored) {
                }
            }
            if (instance != null) {
                try {
                    instance.getClass().getMethod("refreshClass", Object.class)
                            .invoke(instance, new BenchPanelsConfig());
                } catch (Throwable ignored) {
                }
            }
        } catch (Throwable ignored) {
        }
        long t1 = BenchUtil.nowNs();

        telemetry.addLine("Calls PanelsConfigurables.refreshClass once in init().");
        telemetry.addData("refreshClass us", String.format("%.1f", BenchUtil.nsToUs(t1 - t0)));
        telemetry.update();
    }

    @Override
    public void loop() {
        driveFromGamepad();
        recordLoopDtMs();
        telemetryLoopStats();
    }
}
