package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;

@TeleOp(name = "PsiKit Linear Session Minimal", group = "Test")
public class PsiKitLinearSessionMinimal extends LinearOpMode {

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    @Override
    public void runOpMode() {
        try {
            // Start PsiKit first so the OpMode's hardwareMap is wrapped.
            // AdvantageScope defaults to port 5800.
            // If another device (e.g., Limelight) is already using 5800/5801, change this port
            // and update the port in AdvantageScope to match.
            psiKit.start(this, 5802);

            telemetry.addLine("PsiKit logging active (LinearOpMode + FtcLoggingSession).");
            telemetry.addLine("Start this OpMode, move sticks, press buttons.");
            telemetry.addLine("Log file: /sdcard/FIRST/PsiKit/<OpMode>_log_<timestamp>.rlog");
            telemetry.update();

            int loopCount = 0;

            while (opModeInInit()) {
                Logger.periodicBeforeUser();
                psiKit.logOncePerLoop(this);


                telemetry.addData("Status", "INIT");
                telemetry.addData("LoopCount", loopCount);
                telemetry.update();

                Logger.periodicAfterUser(0.0, 0.0);
            }

            waitForStart();

            loopCount = 0;

            while (opModeIsActive()) {
                double beforeUserStart = Logger.getTimestamp();

                Logger.periodicBeforeUser();
                psiKit.logOncePerLoop(this);

                double beforeUserEnd = Logger.getTimestamp();

                // USER CODE GOES HERE (read gamepads, update subsystems, etc.)

                Logger.recordOutput("Example/LoopCount", loopCount++);
                Logger.recordOutput("Example/Gamepad1LeftY", (double) gamepad1.left_stick_y);

                telemetry.addData("Status", "RUN");
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
            psiKit.end();
        }
    }
}
