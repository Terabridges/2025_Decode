package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;

/**
 * Logs FTC {@link Gamepad} state using a schema that AdvantageScope's "Joysticks" view recognizes.
 *
 * <p>AdvantageScope expects a joystick entry under paths like {@code DriverStation/Joystick0}
 * with keys like {@code axisValues}, {@code buttonValues}, and {@code povs}. PsiKit's built-in
 * FTC {@code GamepadWrapper} uses different key names and paths, so the data appears in the raw
 * field list but not in the Joysticks visualization.
 */
public final class AdvantageScopeJoystickInputs implements LoggableInputs {
    private static final int BUTTON_COUNT = 12;

    private final float[] axisValues = new float[6];
    private int buttonValues = 0;
    private final int[] povs = new int[] { -1 };

    /** Updates the stored values from the current gamepad snapshot. */
    public void updateFrom(Gamepad gamepad) {
        if (gamepad == null) {
            axisValues[0] = 0;
            axisValues[1] = 0;
            axisValues[2] = 0;
            axisValues[3] = 0;
            axisValues[4] = 0;
            axisValues[5] = 0;
            buttonValues = 0;
            povs[0] = -1;
            return;
        }

        // Typical joystick axis order:
        // 0: LX, 1: LY, 2: LT, 3: RT, 4: RX, 5: RY
        axisValues[0] = gamepad.left_stick_x;
        axisValues[1] = gamepad.left_stick_y;
        axisValues[2] = gamepad.left_trigger;
        axisValues[3] = gamepad.right_trigger;
        axisValues[4] = gamepad.right_stick_x;
        axisValues[5] = gamepad.right_stick_y;

        // POV (dpad). AdvantageScope expects degrees or -1.
        if (gamepad.dpad_up) {
            povs[0] = 0;
        } else if (gamepad.dpad_right) {
            povs[0] = 90;
        } else if (gamepad.dpad_down) {
            povs[0] = 180;
        } else if (gamepad.dpad_left) {
            povs[0] = 270;
        } else {
            povs[0] = -1;
        }

        // buttonValues bit i corresponds to button (i+1).
        int buttons = 0;
        buttons |= bit(gamepad.a, 0);
        buttons |= bit(gamepad.b, 1);
        buttons |= bit(gamepad.x, 2);
        buttons |= bit(gamepad.y, 3);
        buttons |= bit(gamepad.left_bumper, 4);
        buttons |= bit(gamepad.right_bumper, 5);
        buttons |= bit(gamepad.back, 6);
        buttons |= bit(gamepad.start, 7);
        buttons |= bit(gamepad.left_stick_button, 8);
        buttons |= bit(gamepad.right_stick_button, 9);
        buttons |= bit(gamepad.guide, 10);
        buttons |= bit(gamepad.touchpad, 11);
        buttonValues = buttons;
    }

    private static int bit(boolean value, int bitIndex) {
        return value ? (1 << bitIndex) : 0;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("ButtonCount", BUTTON_COUNT);
        table.put("ButtonValues", buttonValues);
        table.put("AxisValues", axisValues);
        table.put("POVs", povs);
    }

    @Override
    public void fromLog(LogTable table) {
        buttonValues = table.get("ButtonValues", 0);
        float[] axes = table.get("AxisValues", new float[] { 0, 0, 0, 0, 0, 0 });
        System.arraycopy(axes, 0, axisValues, 0, Math.min(axes.length, axisValues.length));
        int[] p = table.get("POVs", new int[] { -1 });
        povs[0] = p.length > 0 ? p[0] : -1;
    }
}
