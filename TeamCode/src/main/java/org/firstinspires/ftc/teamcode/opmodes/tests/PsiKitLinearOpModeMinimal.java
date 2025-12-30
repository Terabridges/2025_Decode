package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.ftc.PsiKitLinearOpMode;

@TeleOp(name = "PsiKit Linear Base Minimal", group = "Test")
public class PsiKitLinearOpModeMinimal extends PsiKitLinearOpMode {

    @Override
    public void runOpMode() {
        try {
            // IMPORTANT: PsiKit wraps `hardwareMap` and `gamepad1/2` in `psiKitSetup()`.
            // Any `hardwareMap.get(...)` calls MUST happen after setup.
            psiKitSetup();

            String filename = this.getClass().getSimpleName()
                    + "_log_"
                    + new java.text.SimpleDateFormat("yyyyMMdd_HHmmss_SSS").format(new java.util.Date())
                    + ".rlog";
            Logger.addDataReceiver(new RLOGWriter(filename));

            Logger.start();

            // INIT loop
            while (opModeInInit() && !isStopRequested()) {
                Logger.periodicBeforeUser();
                processHardwareInputs();

                telemetry.addLine("PsiKit logging active (PsiKitLinearOpMode base).");
                telemetry.addLine("Press START to begin.");
                telemetry.addLine("Log file: /sdcard/FIRST/PsiKit/<OpMode>_log_<timestamp>.rlog");
                telemetry.update();

                Logger.periodicAfterUser(0.0, 0.0);
                idle();
            }

            waitForStart();

            int loopCount = 0;

            while (opModeIsActive() && !isStopRequested()) {
                double beforeUserStart = Logger.getTimestamp();
                Logger.periodicBeforeUser();
                double beforeUserEnd = Logger.getTimestamp();

                processHardwareInputs();

                // USER CODE GOES HERE
                Logger.recordOutput("Example/LoopCount", loopCount++);
                Logger.recordOutput("Example/Gamepad1LeftY", (double) gamepad1.left_stick_y);

                telemetry.addData("LoopCount", loopCount);
                telemetry.update();

                double afterUserStart = Logger.getTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );

                idle();
            }
        } finally {
            try {
                Logger.end();
            } catch (Throwable ignored) {
            }
        }
    }
}
