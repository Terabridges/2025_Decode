package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bench0: DriveOnly (no PsiKit)", group = "Bench")
public class Bench0_DriveOnly extends BenchBaseDriveOpMode {

    @Override
    protected String getBenchName() {
        return "Bench0: DriveOnly (no PsiKit)";
    }

    @Override
    public void loop() {
        maybeClearBulkCache();
        driveFromGamepad();
        recordLoopDtMs();
        telemetryLoopStats();
    }
}
