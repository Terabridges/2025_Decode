package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Minimal motor logging via PsiKit outputs.
 *
 * <p>We use {@link Logger#recordOutput} (not auto-wrappers) so we can log values like motor
 * current that aren't necessarily captured by PsiKit's FTC wrappers.
 */
public final class PsiKitMotorLogger {

    private final List<NamedMotor> cachedMotors = new ArrayList<>();
    private boolean cached;

    /**
     * Logs a small, high-signal set of fields for a motor.
     */
    public void log(String motorName, DcMotorEx motor) {
        logMotor(motorName, motor);
    }

    /**
     * Dynamically logs all configured motors found in {@link HardwareMap}.
     *
     * <p>This avoids having to name motors in your OpMode.
     *
     * <p>Call after {@code psiKitSetup()} so any PsiKit wrapping has already happened.
     */
    public void logAll(HardwareMap hardwareMap) {
        if (!cached) {
            cacheMotors(hardwareMap);
            cached = true;
        }

        for (NamedMotor named : cachedMotors) {
            logMotor(named.name, named.motor);
        }
    }

    private void cacheMotors(HardwareMap hardwareMap) {
        cachedMotors.clear();

        // Prefer DcMotorEx directly. In some setups, getAll(DcMotor.class) can surface
        // wrapper/proxy instances that return stubbed values (notably current).
        List<DcMotorEx> exMotors = new ArrayList<>();
        try {
            exMotors = hardwareMap.getAll(DcMotorEx.class);
        } catch (Throwable ignored) {
            // Some SDK variants may not support getAll for DcMotorEx; fall back below.
        }

        if (exMotors != null && !exMotors.isEmpty()) {
            for (DcMotorEx motorEx : exMotors) {
                String chosenName = firstNameOrFallback(hardwareMap, motorEx, "motor");
                cachedMotors.add(new NamedMotor(chosenName, motorEx));
            }
            return;
        }

        // Fallback: enumerate DcMotor and keep those that also implement DcMotorEx.
        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        for (DcMotor dcMotor : motors) {
            if (!(dcMotor instanceof DcMotorEx)) {
                continue;
            }
            DcMotorEx motorEx = (DcMotorEx) dcMotor;
            String chosenName = firstNameOrFallback(hardwareMap, dcMotor, "motor");
            cachedMotors.add(new NamedMotor(chosenName, motorEx));
        }
    }

    private static String firstNameOrFallback(HardwareMap hardwareMap, HardwareDevice device, String fallbackPrefix) {
        try {
            Set<String> names = hardwareMap.getNamesOf(device);
            if (names != null && !names.isEmpty()) {
                return names.iterator().next();
            }
        } catch (Throwable ignored) {
            // Some SDK variants may not support getNamesOf; fall back.
        }
        return fallbackPrefix;
    }

    private static void logMotor(String motorName, DcMotorEx motor) {
        String base = "Motors/" + motorName + "/";

        Logger.recordOutput(base + "Power", motor.getPower());
        Logger.recordOutput(base + "PositionTicks", motor.getCurrentPosition());
        Logger.recordOutput(base + "TargetPositionTicks", motor.getTargetPosition());

        Logger.recordOutput(base + "VelocityTicksPerSec", motor.getVelocity());
        Logger.recordOutput(base + "CurrentAmps", readCurrentAmps(motor));

        Logger.recordOutput(base + "IsOverCurrent", motor.isOverCurrent() ? 1.0 : 0.0);
    }

    private static double readCurrentAmps(DcMotorEx motor) {
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Throwable ignored) {
            return 0.0;
        }
    }

    private static final class NamedMotor {
        private final String name;
        private final DcMotorEx motor;

        private NamedMotor(String name, DcMotorEx motor) {
            this.name = name;
            this.motor = motor;
        }
    }
}
