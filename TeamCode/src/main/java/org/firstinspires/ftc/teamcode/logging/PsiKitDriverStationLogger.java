package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.psilynx.psikit.core.LoggableInputs;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;

/**
 * Centralizes PsiKit logging for Driver Station inputs.
 *
 * <p>Why this exists:
 * <ul>
 *   <li>PsiKit wraps {@code gamepad1/2} in {@code psiKitSetup()}, but the FTC SDK can still
 *       provide a plain {@link Gamepad} instance (or swap instances). Casting can crash.</li>
 *   <li>PsiKit does not automatically log gamepad state; we call {@link Logger#processInputs}
 *       explicitly.</li>
 *   <li>AdvantageScope's Joysticks tab expects keys under {@code /DriverStation/JoystickN/...}.</li>
 * </ul>
 */
public final class PsiKitDriverStationLogger {

    private Gamepad lastRawGamepad1;
    private LoggableInputs cachedRawGamepad1Inputs;

    private Gamepad lastRawGamepad2;
    private LoggableInputs cachedRawGamepad2Inputs;

    private final AdvantageScopeJoystickInputs joystick0 = new AdvantageScopeJoystickInputs();
    private final AdvantageScopeJoystickInputs joystick1 = new AdvantageScopeJoystickInputs();

    /**
     * Logs both PsiKit gamepad fields and AdvantageScope-compatible joystick fields.
     * Call once per loop, after {@code Logger.periodicBeforeUser()}.
     */
    public void log(Gamepad gamepad1, Gamepad gamepad2) {
        Logger.processInputs("DriverStation/Gamepad1", asLoggableGamepad1(gamepad1));
        Logger.processInputs("DriverStation/Gamepad2", asLoggableGamepad2(gamepad2));

        joystick0.updateFrom(gamepad1);
        joystick1.updateFrom(gamepad2);
        Logger.processInputs("/DriverStation/Joystick0", joystick0);
        Logger.processInputs("/DriverStation/Joystick1", joystick1);
    }

    private LoggableInputs asLoggableGamepad1(Gamepad gamepad) {
        if (gamepad instanceof LoggableInputs) {
            return (LoggableInputs) gamepad;
        }
        if (gamepad != lastRawGamepad1 || cachedRawGamepad1Inputs == null) {
            lastRawGamepad1 = gamepad;
            cachedRawGamepad1Inputs = new GamepadWrapper(gamepad);
        }
        return cachedRawGamepad1Inputs;
    }

    private LoggableInputs asLoggableGamepad2(Gamepad gamepad) {
        if (gamepad instanceof LoggableInputs) {
            return (LoggableInputs) gamepad;
        }
        if (gamepad != lastRawGamepad2 || cachedRawGamepad2Inputs == null) {
            lastRawGamepad2 = gamepad;
            cachedRawGamepad2Inputs = new GamepadWrapper(gamepad);
        }
        return cachedRawGamepad2Inputs;
    }
}
