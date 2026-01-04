package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bench0b: DriveOnly + ManualBulk (no PsiKit)", group = "Bench")
public class Bench0b_DriveOnly_ManualBulk extends BenchBaseDriveOpMode {

    public Bench0b_DriveOnly_ManualBulk() {
        useManualBulkCaching = true;
    }

    @Override
    protected String getBenchName() {
        return "Bench0b: DriveOnly + ManualBulk (no PsiKit)";
    }

    @Override
    public void loop() {
        maybeClearBulkCache();
        driveFromGamepad();
        recordLoopDtMs();
        telemetryLoopStats();
    }
}
