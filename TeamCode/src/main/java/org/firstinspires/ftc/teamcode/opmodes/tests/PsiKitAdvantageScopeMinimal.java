package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;

@TeleOp(name = "PsiKit AdvantageScope Minimal", group = "Test")
public class PsiKitAdvantageScopeMinimal extends LinearOpMode {

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    @Override
    public void runOpMode() {
        try {
            // Start PsiKit first so the OpMode's hardwareMap is wrapped.
            // AdvantageScope defaults to port 5800.
            // If another device (e.g., Limelight) is already using 5800/5801, change this port
            // and update the port in AdvantageScope to match.
            psiKit.start(this, 5800);

            while (opModeInInit()) {
                Logger.periodicBeforeUser();
                psiKit.logOncePerLoop(this);

                telemetry.addLine("PsiKit logging active.");
                telemetry.addLine("Start this OpMode, move sticks, press buttons.");
                telemetry.addLine("Log file: /sdcard/FIRST/PsiKit/<OpMode>_log_<timestamp>.rlog");
                telemetry.update();

                Logger.periodicAfterUser(0.0, 0.0);
            }

            waitForStart();

            int loopCount = 0;

            while (opModeIsActive()) {
                double beforeUserStart = Logger.getTimestamp();

                Logger.periodicBeforeUser();
                psiKit.logOncePerLoop(this);

                double beforeUserEnd = Logger.getTimestamp();

                // USER CODE GOES HERE (read gamepads, update subsystems, etc.)

                Logger.recordOutput("Example/LoopCount", loopCount++);

                double afterUserStart = Logger.getTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );

                idle();
            }
        } finally {
            psiKit.end();
        }
    }
}
