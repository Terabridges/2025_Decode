package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGReplay;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicLong;

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

        // Allow replay to install a mock/wrapped HardwareMap for hardwareMap.get(...).
        System.setProperty("psikitReplayMockHardwareMap", "true");

        // Reset between runs so loopCount reflects only this test.
        ReplayOpMode.loopCount = 0;

        ReplayOpMode opMode = new ReplayOpMode();
        opMode.hardwareMap = null;
        opMode.gamepad1 = new GamepadWrapper(null);
        opMode.gamepad2 = new GamepadWrapper(null);

        FtcLoggingSession session = new FtcLoggingSession();
        session.start(opMode, 0, "", "/sdcard/FIRST/PsiKit/", replaySource);
        Logger.periodicAfterUser(0.0, 0.0);

        try {
            final double startTs = Logger.getTimestamp();
            while (Logger.getTimestamp() < startTs + 5.0) {
                Logger.periodicBeforeUser();
                session.logOncePerLoop(opMode);

                // USER CODE WOULD RUN HERE
                // (read gamepads / update subsystems / record outputs)
                ReplayOpMode.loopCount++;

                Logger.periodicAfterUser(0.0, 0.0);
            }
        } finally {
            session.end();
        }

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

        // Keep the test self-contained: generate a tiny log if no fixture is available.
        return generateFallbackReplayLog();
    }

    private static Path generateFallbackReplayLog() {
        try {
            Path tempDir = Files.createTempDirectory("psikit-replay-fixture");
            String fileName = "generated_fixture.rlog";
            Path logPath = tempDir.resolve(fileName);

            Logger.reset();

            AtomicLong tick = new AtomicLong(0);
            final double dtSeconds = 0.1; // 10Hz
            Logger.setTimeSource(() -> tick.get() * dtSeconds);

            String dir = tempDir.toString().replace("\\", "/") + "/";
            Logger.addDataReceiver(new RLOGWriter(dir, fileName));

            Logger.start();
            for (int i = 0; i < 70; i++) { // >5s of data
                Logger.periodicAfterUser(0.0, 0.0);
                tick.incrementAndGet();
                Logger.periodicBeforeUser();
            }
            Logger.periodicAfterUser(0.0, 0.0);
            Logger.end();

            if (!Files.exists(logPath)) {
                throw new IllegalStateException("Generated replay log was not created: " + logPath);
            }
            return logPath;
        } catch (Exception e) {
            throw new IllegalStateException(
                    "Could not find replay log fixture and failed to generate one. "
                            + "Set -D" + LOG_PATH_PROPERTY + "=... or env " + LOG_PATH_ENV + ".",
                    e
            );
        }
    }

    @TeleOp(name = "PsiKit Replay Test", group = "Test")
    public static class ReplayOpMode extends OpMode {
        static volatile int loopCount = 0;

        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }
}
