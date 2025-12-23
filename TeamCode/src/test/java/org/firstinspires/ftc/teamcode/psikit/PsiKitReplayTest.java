package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGReplay;
import org.psilynx.psikit.ftc.DriverStationLogger;
import org.psilynx.psikit.ftc.PsiKitLinearOpMode;
import org.psilynx.psikit.ftc.Replay;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

@Config(shadows = {ShadowAppUtil.class})
@RunWith(RobolectricTestRunner.class)
public class PsiKitReplayTest {

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
    public void replayFromFile() {
        Path logPath = resolveReplayLogPath();
        RLOGReplay replaySource = new RLOGReplay(logPath.toString());

        Replay replay = new Replay(new ReplayOpMode(), replaySource);
        replay.run();

        // Basic sanity: the OpMode should have advanced through at least one loop.
        if (ReplayOpMode.loopCount <= 0) {
            throw new AssertionError("Replay loop did not run (loopCount=" + ReplayOpMode.loopCount + ")");
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

        // Gradle unit tests often run with user.dir set to the module directory (TeamCode).
        Path userDir = Paths.get(Objects.requireNonNull(System.getProperty("user.dir")));

        Path[] candidates = new Path[] {
                userDir.resolve(DEFAULT_LOG_FILENAME),
                userDir.getParent() != null ? userDir.getParent().resolve(DEFAULT_LOG_FILENAME) : null,
                // Common when running from workspace root:
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

    @TeleOp(name = "PsiKit Replay Test", group = "Test")
    public static class ReplayOpMode extends PsiKitLinearOpMode {

        private final DriverStationLogger dsLogger = new DriverStationLogger();
        static volatile int loopCount = 0;

        @Override
        public void runOpMode() {
            psiKitSetup();

            Logger.start();
            Logger.periodicAfterUser(0.0, 0.0);

            waitForStart();

            // Replay for a short, fixed time window relative to the log's start time.
            final double startTs = Logger.getTimestamp();
            while (Logger.getTimestamp() < startTs + 5.0) {
                Logger.periodicBeforeUser();

                // Update wrapped hardware state (if any devices are accessed).
                processHardwareInputs();

                // Replay Driver Station inputs in AdvantageScope Joysticks schema.
                dsLogger.log(gamepad1, gamepad2);

                // USER CODE WOULD RUN HERE
                // (read gamepads / update subsystems / record outputs)

                loopCount++;

                Logger.periodicAfterUser(0.0, 0.0);
            }
        }
    }
}
