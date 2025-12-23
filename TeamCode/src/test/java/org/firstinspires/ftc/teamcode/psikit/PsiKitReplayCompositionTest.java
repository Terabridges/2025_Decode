package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.LogReplaySource;
import org.psilynx.psikit.core.rlog.RLOGReplay;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

@Config(shadows = {ShadowAppUtil.class})
@RunWith(RobolectricTestRunner.class)
public class PsiKitReplayCompositionTest {

    /**
     * Override the log path when running tests:
     * - JVM arg: -DpsikitReplayLog=C:\\path\\to\\log.rlog
     * - env var: PSIKIT_REPLAY_LOG=C:\\path\\to\\log.rlog
     */
    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String LOG_PATH_ENV = "PSIKIT_REPLAY_LOG";

    // Pick a repo-committed example log by default.
    private static final String DEFAULT_LOG_FILENAME = "MainTeleopLogging_log_20251220_014453_861.rlog";

    @Test
    public void replayFromFile_compositionStyle_standardLinearOpMode() {
        Path logPath = resolveReplayLogPath();
        RLOGReplay replaySource = new RLOGReplay(logPath.toString());

        StandardReplayOpMode opMode = new StandardReplayOpMode(replaySource);
        opMode.runOpMode();

        if (StandardReplayOpMode.loopCount <= 0) {
            throw new AssertionError(
                    "Replay loop did not run (loopCount=" + StandardReplayOpMode.loopCount + ")"
            );
        }
    }

    private static Path resolveReplayLogPath() {
        String fromProp = System.getProperty(LOG_PATH_PROPERTY);
        if (fromProp != null && !fromProp.trim().isEmpty()) {
            Path p = Paths.get(fromProp.trim());
            if (!Files.exists(p)) {
                throw new IllegalArgumentException("psikitReplayLog does not exist: " + p);
            }
            return p;
        }

        String fromEnv = System.getenv(LOG_PATH_ENV);
        if (fromEnv != null && !fromEnv.trim().isEmpty()) {
            Path p = Paths.get(fromEnv.trim());
            if (!Files.exists(p)) {
                throw new IllegalArgumentException("PSIKIT_REPLAY_LOG does not exist: " + p);
            }
            return p;
        }

        Path userDir = Paths.get(Objects.requireNonNull(System.getProperty("user.dir")));

        Path[] candidates = new Path[] {
                userDir.resolve(DEFAULT_LOG_FILENAME),
                userDir.getParent() != null ? userDir.getParent().resolve(DEFAULT_LOG_FILENAME) : null,
                Paths.get("c:\\code\\TeraBridges\\2025_Decode").resolve(DEFAULT_LOG_FILENAME),
        };

        for (Path candidate : candidates) {
            if (candidate != null && Files.exists(candidate)) {
                return candidate;
            }
        }

        throw new IllegalStateException(
                "Could not find replay log. Set -D" + LOG_PATH_PROPERTY + "=... or env " + LOG_PATH_ENV + ". "
                        + "Tried: " + java.util.Arrays.toString(candidates)
        );
    }

    @TeleOp(name = "PsiKit Replay Composition Test", group = "Test")
    public static class StandardReplayOpMode extends LinearOpMode {

        private final LogReplaySource replaySource;
        private final FtcLoggingSession session = new FtcLoggingSession();

        static volatile int loopCount = 0;

        public StandardReplayOpMode(LogReplaySource replaySource) {
            this.replaySource = replaySource;
        }

        @Override
        public void runOpMode() {
            // Minimal replay wiring similar to PsiKit's Replay helper, but without subclassing PsiKitLinearOpMode.
            hardwareMap = null;
            gamepad1 = new GamepadWrapper(null);
            gamepad2 = new GamepadWrapper(null);

            // In unit tests we typically don't have real hardware; avoid Pinpoint scan.
            session.enablePinpointOdometryLogging = false;

            Logger.setReplay(true);
            Logger.setReplaySource(replaySource);

            Logger.start();
            Logger.periodicAfterUser(0.0, 0.0);

            // Replay for a short, fixed time window relative to the log's start time.
            final double startTs = Logger.getTimestamp();
            while (Logger.getTimestamp() < startTs + 5.0) {
                Logger.periodicBeforeUser();

                // Composition-style: one call per loop to log inputs (DS + any accessed hardware).
                session.logOncePerLoop(this);

                // USER CODE WOULD RUN HERE
                // (read gamepads / update subsystems / record outputs)

                loopCount++;
                Logger.periodicAfterUser(0.0, 0.0);
            }

            session.end();
        }
    }
}
