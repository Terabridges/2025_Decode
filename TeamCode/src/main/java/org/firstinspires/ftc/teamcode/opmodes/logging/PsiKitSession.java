package org.firstinspires.ftc.teamcode.opmodes.logging;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.logging.PsiKitDriverStationLogger;
import org.firstinspires.ftc.teamcode.logging.PsiKitMotorLogger;
import org.firstinspires.ftc.teamcode.logging.PsiKitPinpointV2Logger;
import org.psilynx.psikit.core.LoggableInputs;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.ftc.HardwareMapWrapper;
import org.psilynx.psikit.ftc.OpModeControls;

import java.lang.reflect.Field;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.lang.reflect.Method;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.MANUAL;

/**
 * Composition-based PsiKit logging helper for FTC {@link LinearOpMode}s.
 *
 * <p>This avoids requiring a custom OpMode base class. Typical usage:
 * <ol>
 *   <li>{@link #captureRawFtcReferences(LinearOpMode)}
 *   <li>Construct robot hardware using the original FTC {@code hardwareMap} / {@code gamepad1/2}
 *   <li>{@link #start(LinearOpMode, int)} (wrap + prime for /HardwareMap logging, start RLOG)
 *   <li>Each loop: {@link #logPsiKitInputsOncePerLoop(LinearOpMode)} and optionally
 *       {@link #logRawHardwareOncePerLoop()}
 *   <li>{@link #end()}
 * </ol>
 */
public final class PsiKitSession {

    private final PsiKitDriverStationLogger driverStationLogger = new PsiKitDriverStationLogger();
    private final PsiKitMotorLogger motorLogger = new PsiKitMotorLogger();
    private final PsiKitPinpointV2Logger pinpointLogger = new PsiKitPinpointV2Logger();

    private HardwareMap rawHardwareMap;
    private Gamepad rawGamepad1;
    private Gamepad rawGamepad2;

    private HardwareMap wrappedHardwareMap;
    private List<LynxModule> allHubs;

    public void captureRawFtcReferences(LinearOpMode opMode) {
        if (rawHardwareMap == null) {
            rawHardwareMap = opMode.hardwareMap;
        }
        if (rawGamepad1 == null) {
            rawGamepad1 = opMode.gamepad1;
        }
        if (rawGamepad2 == null) {
            rawGamepad2 = opMode.gamepad2;
        }
    }

    public HardwareMap getRawHardwareMap() {
        return rawHardwareMap;
    }

    public Gamepad getRawGamepad1() {
        return rawGamepad1;
    }

    public Gamepad getRawGamepad2() {
        return rawGamepad2;
    }

    public void start(LinearOpMode opMode, int rlogPort) {
        start(opMode, rlogPort, defaultLogFilename(opMode));
    }

    public void start(LinearOpMode opMode, int rlogPort, String filename) {
        // If the prior OpMode was force-stopped, PsiKit may still be "running".
        try { Logger.end(); } catch (Exception ignored) {}
        Logger.reset();

        // Wrap hardwareMap for /HardwareMap/... inputs.
        opMode.hardwareMap = new HardwareMapWrapper(opMode.hardwareMap);
        wrappedHardwareMap = opMode.hardwareMap;

        // Configure Lynx bulk caching like PsiKitLinearOpMode.
        try {
            allHubs = wrappedHardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(MANUAL);
            }
        } catch (Throwable ignored) {
            allHubs = null;
        }

        // Prime /HardwareMap logging: wrapper needs to see getAll()/get() calls.
        primePsiKitHardwareLogging();

        // Record basic OpMode metadata like PsiKit's base classes do.
        recordOpModeMetadata(opMode);

        Logger.addDataReceiver(new RLOGServer(rlogPort));
        Logger.addDataReceiver(new RLOGWriter(filename));

        if (Logger.isReplay()) {
            // Best-effort: avoid blocking forever in waitForStart()/opModeInInit().
            forceOpModeStarted(opMode);
        }

        Logger.start();
    }

    public void end() {
        try { Logger.end(); } catch (Exception ignored) {}
    }

    /** Call once per loop, after {@link Logger#periodicBeforeUser()}. */
    public void logPsiKitInputsOncePerLoop(LinearOpMode opMode) {
        processHardwareInputs(opMode);
        driverStationLogger.log(opMode.gamepad1, opMode.gamepad2);
    }

    /** Logs motors + pinpoint from raw (unwrapped) hardware to avoid wrapper side effects. */
    public void logRawHardwareOncePerLoop() {
        if (rawHardwareMap == null) {
            return;
        }
        motorLogger.logAll(rawHardwareMap);
        pinpointLogger.logAll(rawHardwareMap);
    }

    private void processHardwareInputs(LinearOpMode opMode) {
        if (allHubs != null) {
            for (LynxModule hub : allHubs) {
                try { hub.clearBulkCache(); } catch (Throwable ignored) {}
            }
        }

        OpModeControls.INSTANCE.setStarted(opMode.isStarted());
        OpModeControls.INSTANCE.setStopped(opMode.isStopRequested());
        Logger.processInputs("OpModeControls", OpModeControls.INSTANCE);

        // HardwareMapWrapper tracks which devices were accessed.
        // Note: this map is Kotlin 'internal', so we access it reflectively.
        Map<String, ?> devices = getDevicesToProcess();
        if (devices != null) {
            for (Map.Entry<String, ?> entry : devices.entrySet()) {
                Object value = entry.getValue();
                if (value instanceof LoggableInputs) {
                    long startNs = System.nanoTime();
                    Logger.processInputs("HardwareMap/" + entry.getKey(), (LoggableInputs) value);
                    long endNs = System.nanoTime();

                    double micros = (endNs - startNs) / 1_000.0;
                    Logger.recordOutput("PsiKit/logTimes (us)/" + entry.getKey(), micros);
                }
            }
        }
    }

    @SuppressWarnings("unchecked")
    private static Map<String, ?> getDevicesToProcess() {
        // Kotlin 'internal' + different build variants can mangle accessors.
        // Be defensive: look for any field/method whose name contains "devicesToProcess".
        try {
            Object companion = HardwareMapWrapper.Companion;
            Map<String, ?> fromCompanion = tryReadDevicesMapFromObject(companion);
            if (fromCompanion != null) {
                return fromCompanion;
            }

            Map<String, ?> fromOuter = tryReadDevicesMapFromClass(HardwareMapWrapper.class);
            if (fromOuter != null) {
                return fromOuter;
            }
        } catch (Throwable ignored) {
            // Fall through.
        }
        return null;
    }

    private static Map<String, ?> tryReadDevicesMapFromObject(Object obj) {
        if (obj == null) {
            return null;
        }

        // Try fields first.
        try {
            for (Field f : obj.getClass().getDeclaredFields()) {
                if (!f.getName().toLowerCase().contains("devicestoprocess")) {
                    continue;
                }
                f.setAccessible(true);
                Object value = f.get(obj);
                if (value instanceof Map) {
                    return (Map<String, ?>) value;
                }
            }
        } catch (Throwable ignored) {
        }

        // Try getters (Kotlin may generate getDevicesToProcess$<module>).
        try {
            for (Method m : obj.getClass().getDeclaredMethods()) {
                if (!m.getName().toLowerCase().startsWith("getdevicestoprocess")) {
                    continue;
                }
                if (m.getParameterTypes().length != 0) {
                    continue;
                }
                m.setAccessible(true);
                Object value = m.invoke(obj);
                if (value instanceof Map) {
                    return (Map<String, ?>) value;
                }
            }
        } catch (Throwable ignored) {
        }

        return null;
    }

    private static Map<String, ?> tryReadDevicesMapFromClass(Class<?> clazz) {
        if (clazz == null) {
            return null;
        }
        try {
            for (Field f : clazz.getDeclaredFields()) {
                if (!f.getName().toLowerCase().contains("devicestoprocess")) {
                    continue;
                }
                f.setAccessible(true);
                Object value = f.get(null);
                if (value instanceof Map) {
                    return (Map<String, ?>) value;
                }
            }
        } catch (Throwable ignored) {
        }
        return null;
    }

    private void primePsiKitHardwareLogging() {
        if (rawHardwareMap == null || wrappedHardwareMap == null) {
            return;
        }

        // Prefer enumerating names from the raw HardwareMap, then re-fetching by name
        // from the wrapped HardwareMap. This avoids relying on HardwareMapWrapper.getAll(),
        // which can throw if getNamesOf(...) returns an empty set.
        primeByType(DcMotor.class);
        primeByType(Servo.class);
        primeByType(CRServo.class);
        primeByType(DigitalChannel.class);
        primeByType(AnalogInput.class);
        primeByType(VoltageSensor.class);
    }

    private <T> void primeByType(Class<T> clazz) {
        try {
            List<T> devices = rawHardwareMap.getAll(clazz);
            for (T device : devices) {
                try {
                    for (String name : rawHardwareMap.getNamesOf((com.qualcomm.robotcore.hardware.HardwareDevice) device)) {
                        if (name == null) {
                            continue;
                        }
                        try {
                            wrappedHardwareMap.get(clazz, name);
                        } catch (Throwable ignored) {
                            // Keep priming others.
                        }
                    }
                } catch (Throwable ignored) {
                    // Keep priming others.
                }
            }
        } catch (Throwable ignored) {
            // Some SDK variants or proxies may not support getAll for this type.
        }
    }

    private static String defaultLogFilename(LinearOpMode opMode) {
        return opMode.getClass().getSimpleName()
                + "_log_"
                + new SimpleDateFormat("yyyyMMdd_HHmmss_SSS").format(new Date())
                + ".rlog";
    }

    private static void recordOpModeMetadata(LinearOpMode opMode) {
        TeleOp teleOp = opMode.getClass().getAnnotation(TeleOp.class);
        if (teleOp != null) {
            Logger.recordMetadata("OpMode Name", teleOp.name());
            Logger.recordMetadata("OpMode type", "TeleOp");
            return;
        }

        Autonomous auto = opMode.getClass().getAnnotation(Autonomous.class);
        if (auto != null) {
            Logger.recordMetadata("OpMode Name", auto.name());
            Logger.recordMetadata("OpMode type", "Autonomous");
            return;
        }

        Logger.recordMetadata("OpMode Name", opMode.getClass().getSimpleName());
        Logger.recordMetadata("OpMode type", "Unknown");
    }

    private static void forceOpModeStarted(LinearOpMode opMode) {
        try {
            Field startedField = OpMode.class.getDeclaredField("isStarted");
            startedField.setAccessible(true);
            startedField.setBoolean(opMode, true);
        } catch (Throwable ignored) {
        }
    }
}
