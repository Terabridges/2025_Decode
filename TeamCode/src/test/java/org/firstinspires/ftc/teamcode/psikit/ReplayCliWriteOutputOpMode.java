package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Replay OpMode used by the CLI runner to validate that replay can also write a new RLOG.
 *
 * Requires:
 * - -DpsikitReplayLog=... (or env PSIKIT_REPLAY_LOG)
 * - -DpsikitReplayWriteOutput=true
 *
 * Optional:
 * - -DpsikitReplayOutputDir=... (defaults to TeamCode/build/psikitReplayOut)
 */
@TeleOp(name = "Replay CLI Write Output OpMode", group = "Test")
public class ReplayCliWriteOutputOpMode extends LinearOpMode {

    private static final String OUT_DIR_PROPERTY = "psikitReplayOutputDir";
    private static final String OUT_DIR_ENV = "PSIKIT_REPLAY_OUTPUT_DIR";

    private final FtcLoggingSession session = new FtcLoggingSession();

    @Override
    public void runOpMode() {
        hardwareMap = null;
        gamepad1 = new GamepadWrapper(null);
        gamepad2 = new GamepadWrapper(null);

        session.enablePinpointOdometryLogging = false;

        // Prefer JVM properties (-D...) but accept env vars as a fallback for Gradle/CI reliability.
        String outDirRaw = System.getProperty(OUT_DIR_PROPERTY);
        if (outDirRaw == null || outDirRaw.trim().isEmpty()) {
            outDirRaw = System.getenv(OUT_DIR_ENV);
        }
        Path outDir = (outDirRaw == null || outDirRaw.trim().isEmpty())
                ? Paths.get("build", "psikitReplayOut")
                : Paths.get(outDirRaw.trim());

        // RLOGWriter expects forward slashes.
        String outDirUnix = outDir.toAbsolutePath().normalize().toString().replace('\\', '/') + "/";
        String filename = "replay_output_from_replay.rlog";

        // Port 0 disables server. During replay, writer is enabled via -DpsikitReplayWriteOutput=true.
        // Replay source is auto-detected by FtcLoggingSession via -DpsikitReplayLog / PSIKIT_REPLAY_LOG.
        session.start(this, 0, filename, outDirUnix, null);
        Logger.periodicAfterUser(0.0, 0.0);

        // Let replay drive init/start transitions.
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

        File outFile = new File(outDirUnix + filename);
        if (!outFile.exists()) {
            throw new AssertionError("Expected output rlog to exist: " + outFile.getAbsolutePath());
        }
        if (outFile.length() <= 0) {
            throw new AssertionError("Expected output rlog to be non-empty: " + outFile.getAbsolutePath());
        }
        if (loops <= 0) {
            throw new AssertionError("Replay loop did not run (loops=" + loops + ")");
        }
    }
}
