package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.junit.Assume;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Command-line runner for replaying real FTC OpModes under Robolectric.
 *
 * This test is skipped unless you provide {@code -DpsikitReplayOpMode=<fqcn>}.
 *
 * Required properties/env:
 * - {@code -DpsikitReplayOpMode=<fully.qualified.OpModeClass>}
 * - {@code -DpsikitReplayLog=<path-to-rlog>} (or env {@code PSIKIT_REPLAY_LOG})
 *
 * Example:
 *   ./gradlew :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.psikit.PsiKitReplayCliRunnerTest \
 *     -DpsikitReplayOpMode=org.firstinspires.ftc.teamcode.psikit.ReplayCliSmokeOpMode \
 *     -DpsikitReplayLog=C:\\path\\to\\log.rlog
 */
@Config(shadows = {ShadowAppUtil.class})
@RunWith(RobolectricTestRunner.class)
public class PsiKitReplayCliRunnerTest {

    private static final String OPMODE_PROPERTY = "psikitReplayOpMode";
    private static final String OPMODE_ENV = "PSIKIT_REPLAY_OPMODE";

    private static final long DEFAULT_TIMEOUT_MS = 120_000;

    private static boolean setBooleanFieldIfPresent(Object target, String fieldName, boolean value) {
        for (Class<?> c = target.getClass(); c != null; c = c.getSuperclass()) {
            try {
                Field f = c.getDeclaredField(fieldName);
                f.setAccessible(true);
                f.setBoolean(target, value);
                return true;
            } catch (NoSuchFieldException ignored) {
                // keep searching
            } catch (Throwable ignored) {
                return false;
            }
        }
        return false;
    }

    private static void forceInitState(LinearOpMode opMode) {
        // FTC SDK: LinearOpMode's opModeInInit()/opModeIsActive() depend on these internal fields.
        // Force a stable INIT state so replay can drive the start transition from the recorded log.
        setBooleanFieldIfPresent(opMode, "isStarted", false);
        setBooleanFieldIfPresent(opMode, "stopRequested", false);
    }

    private static void installNoOpInternalOpModeServices(LinearOpMode opMode) {
        try {
            Field internalServicesField = com.qualcomm.robotcore.eventloop.opmode.OpMode.class
                    .getDeclaredField("internalOpModeServices");
            internalServicesField.setAccessible(true);

            Class<?> servicesClass = internalServicesField.getType();
            if (!servicesClass.isInterface()) {
                return;
            }

            Object servicesProxy = Proxy.newProxyInstance(
                    servicesClass.getClassLoader(),
                    new Class<?>[]{servicesClass},
                    (proxy, method, args) -> {
                        Class<?> returnType = method.getReturnType();
                        if (returnType == boolean.class) return false;
                        if (returnType == byte.class) return (byte) 0;
                        if (returnType == short.class) return (short) 0;
                        if (returnType == int.class) return 0;
                        if (returnType == long.class) return 0L;
                        if (returnType == float.class) return 0f;
                        if (returnType == double.class) return 0d;
                        if (returnType == char.class) return (char) 0;
                        return null;
                    }
            );
            internalServicesField.set(opMode, servicesProxy);
        } catch (Throwable ignored) {
            // Best-effort: some SDK versions may move/rename internals. If we can't install this,
            // OpModes that call telemetry.update() may still fail under replay.
        }
    }

    private static void forceStopRequested(LinearOpMode opMode) {
        // Fallback for JVM replay: requestOpModeStop() can NPE if internal SDK services aren't set.
        // We try to flip any "stop" booleans/AtomicBooleans on the opmode/parents so opModeIsActive()
        // loops can exit.
        for (Class<?> c = opMode.getClass(); c != null; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                String name = f.getName() == null ? "" : f.getName().toLowerCase();
                try {
                    if (f.getType() == boolean.class && name.contains("stop")) {
                        f.setAccessible(true);
                        f.setBoolean(opMode, true);
                    } else if (java.util.concurrent.atomic.AtomicBoolean.class.isAssignableFrom(f.getType())
                            && name.contains("stop")) {
                        f.setAccessible(true);
                        Object v = f.get(opMode);
                        if (v instanceof java.util.concurrent.atomic.AtomicBoolean) {
                            ((java.util.concurrent.atomic.AtomicBoolean) v).set(true);
                        }
                    }
                } catch (Throwable ignored) {
                    // Best effort
                }
            }
        }
    }

    @Test
    public void runReplayOpModeFromCommandLine() throws Exception {
        // Prefer JVM properties (-D...) for IDE/per-invocation overrides, but also accept env vars
        // because Gradle/test workers/Windows quoting can make -D propagation unreliable.
        String opModeClassName = System.getProperty(OPMODE_PROPERTY);
        if (opModeClassName == null || opModeClassName.trim().isEmpty()) {
            opModeClassName = System.getenv(OPMODE_ENV);
        }
        Assume.assumeTrue(
            "Skipping: set -D" + OPMODE_PROPERTY + "=<fully.qualified.OpModeClass> or env " + OPMODE_ENV,
                opModeClassName != null && !opModeClassName.trim().isEmpty()
        );

        Class<?> clazz = Class.forName(opModeClassName.trim());
        if (!LinearOpMode.class.isAssignableFrom(clazz)) {
            throw new IllegalArgumentException(
                    "Class does not extend LinearOpMode: " + opModeClassName
            );
        }

        LinearOpMode opMode;
        try {
            opMode = (LinearOpMode) clazz.getDeclaredConstructor().newInstance();
        } catch (NoSuchMethodException noNoArgCtor) {
            throw new IllegalArgumentException(
                    "OpMode must have a public no-arg constructor for CLI replay: " + opModeClassName,
                    noNoArgCtor
            );
        }

        // Minimal wiring consistent with existing replay unit tests.
        // Many OpModes will work in replay if they are PsiKit-instrumented and tolerate null hardwareMap.
        opMode.hardwareMap = null;
        opMode.gamepad1 = new GamepadWrapper(null);
        opMode.gamepad2 = new GamepadWrapper(null);

        // Ensure opModeInInit() is true at the start so replay can drive start/stop transitions
        // from the recorded OpModeControls.
        forceInitState(opMode);

        // Needed for OpModes that call telemetry.update(); avoids NPE in FTC internals.
        installNoOpInternalOpModeServices(opMode);

        // Run the OpMode on a worker thread so we can stop it once the replay ends.
        AtomicReference<Throwable> opModeFailure = new AtomicReference<>(null);
        Thread opModeThread = new Thread(() -> {
            try {
                opMode.runOpMode();
            } catch (Throwable t) {
                opModeFailure.set(t);
            }
        }, "PsiKitReplayOpModeThread");

        opModeThread.start();

        long deadlineMs = System.currentTimeMillis() + DEFAULT_TIMEOUT_MS;
        boolean stopRequested = false;
        boolean loggerEverRunning = false;
        while (opModeThread.isAlive() && System.currentTimeMillis() < deadlineMs) {
            loggerEverRunning |= org.psilynx.psikit.core.Logger.isRunning();

            // When the replay source hits EOF, Logger.end() is called and Logger.isRunning()
            // becomes false. Many LinearOpModes use opModeIsActive() loops that won't exit
            // unless stop is requested, so we request stop here.
            // Important: avoid requesting stop during PsiKit setup (Logger.isReplay may become
            // true before Logger.start sets running=true).
            if (!stopRequested
                    && loggerEverRunning
                    && org.psilynx.psikit.core.Logger.isReplay()
                    && !org.psilynx.psikit.core.Logger.isRunning()) {
                try {
                    opMode.requestOpModeStop();
                } catch (Throwable t) {
                    forceStopRequested(opMode);
                }
                stopRequested = true;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        if (opModeThread.isAlive()) {
            try {
                opMode.requestOpModeStop();
            } catch (Throwable t) {
                forceStopRequested(opMode);
            }
            opModeThread.interrupt();
            throw new AssertionError("OpMode did not exit within timeout (" + DEFAULT_TIMEOUT_MS + "ms)");
        }

        Throwable failure = opModeFailure.get();
        if (failure != null) {
            throw new AssertionError("OpMode crashed during replay", failure);
        }
    }
}
