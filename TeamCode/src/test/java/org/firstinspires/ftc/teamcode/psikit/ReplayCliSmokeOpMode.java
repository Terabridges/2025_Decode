package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;

/**
 * Minimal OpMode for validating command-line replay.
 *
 * This relies on PsiKit's FtcLoggingSession.start() auto-detecting -DpsikitReplayLog
 * and configuring replay.
 */
@TeleOp(name = "Replay CLI Smoke OpMode", group = "Test")
public class ReplayCliSmokeOpMode extends LinearOpMode {

    private final FtcLoggingSession session = new FtcLoggingSession();

    @Override
    public void runOpMode() {
        hardwareMap = null;
        gamepad1 = new GamepadWrapper(null);
        gamepad2 = new GamepadWrapper(null);

        session.enablePinpointOdometryLogging = false;

        // Port 0 disables RLOGServer. In replay, file output is disabled by default.
        session.start(this, 0);
        Logger.periodicAfterUser(0.0, 0.0);

        // Allow replay to drive init->start transitions.
        final int maxInitIterations = 5000;
        int initIterations = 0;
        while (opModeInInit()) {
            if (initIterations++ > maxInitIterations) {
                throw new AssertionError("Init loop exceeded max iterations (" + maxInitIterations + ")");
            }
            Logger.periodicBeforeUser();
            session.logOncePerLoop(this);
            Logger.periodicAfterUser(0.0, 0.0);

            if (!Logger.isRunning()) {
                throw new AssertionError("Replay ended during init before started=true");
            }
        }

        waitForStart();

        final double startTs = Logger.getTimestamp();
        if (!Double.isFinite(startTs)) {
            throw new AssertionError("Non-finite replay timestamp: startTs=" + startTs);
        }

        // Run until the replay source ends the log (Logger.isRunning() becomes false).
        // Keep a high iteration cap as a safety net for stuck replays.
        final int maxLoopIterations = 5_000_000;
        int loops = 0;
        while (Logger.isRunning()) {
            if (loops++ > maxLoopIterations) {
                throw new AssertionError("Main loop exceeded max iterations (" + maxLoopIterations + ")");
            }
            Logger.periodicBeforeUser();
            if (!Logger.isRunning()) {
                break;
            }
            session.logOncePerLoop(this);
            Logger.periodicAfterUser(0.0, 0.0);
        }

        session.end();

        if (loops <= 0) {
            throw new AssertionError("Replay loop did not run (loops=" + loops + ")");
        }
    }
}
