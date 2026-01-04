package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bench4b: Limelight (fetched only)", group = "Bench")
public class Bench4b_Limelight_FetchedOnly extends BenchBaseDriveOpMode {

    private Limelight3A limelight;

    @Override
    protected String getBenchName() {
        return "Bench4b: Limelight (fetched only)";
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
        telemetry.addLine("Does NOT start or poll Limelight; measures just device fetch + app background.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveFromGamepad();
        recordLoopDtMs();
        telemetryLoopStats();
    }
}
