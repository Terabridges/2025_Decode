package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;

@TeleOp(name = "PsiKit Iterative Session Minimal", group = "Test")
public class PsiKitOpModeSessionMinimal extends OpMode {

    private static final int RLOG_PORT = 5802;  //AdvantageScope defaults to port 5800, but many devices (e.g. Limelight) use 5800/5801

    private final FtcLoggingSession psiKit = new FtcLoggingSession();
    private boolean psiKitStarted = false;

    private int loopCount = 0;

    @Override
    public void init() {
        // Start PsiKit first so the OpMode's hardwareMap + gamepads are wrapped.
        if (!psiKitStarted) {
            psiKit.start(this, RLOG_PORT);
            psiKitStarted = true;
        }

        telemetry.addLine("PsiKit logging active (OpMode + FtcLoggingSession).");
        telemetry.addLine("Log file: /sdcard/FIRST/PsiKit/<OpMode>_log_<timestamp>.rlog");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Logger.periodicBeforeUser();
        psiKit.logOncePerLoop(this);

        telemetry.addData("Status", "INIT");
        telemetry.addData("LoopCount", loopCount);
        telemetry.update();

        Logger.periodicAfterUser(0.0, 0.0);
    }

    @Override
    public void start() {
        loopCount = 0;
    }

    @Override
    public void loop() {
        double beforeUserStart = Logger.getRealTimestamp();

        Logger.periodicBeforeUser();
        psiKit.logOncePerLoop(this);

        double beforeUserEnd = Logger.getRealTimestamp();

        // USER CODE GOES HERE (read gamepads, update subsystems, etc.)
        Logger.recordOutput("Example/LoopCount", loopCount++);
        Logger.recordOutput("Example/Gamepad1LeftY", (double) gamepad1.left_stick_y);

        telemetry.addData("Status", "RUN");
        telemetry.addData("LoopCount", loopCount);
        telemetry.update();

        double afterUserStart = Logger.getRealTimestamp();
        Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
        );
    }

    @Override
    public void stop() {
        try {
            // No-op for user code.
        } finally {
            psiKit.end();
            psiKitStarted = false;
        }
    }
}
