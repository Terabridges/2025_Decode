package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;

/**
 * AdvantageScope Joysticks schema logger (WPILib/AdvantageKit-style).
 *
 * <p>Keys under {@code /DriverStation/JoystickN/...} populate AdvantageScope's Joysticks tab.
 */
public final class AdvantageScopeJoystickInputs implements LoggableInputs {
    private static final int BUTTON_COUNT = 12;

    // Matches AdvantageKit/WPILib "DriverStation/JoystickN" schema
    private final double[] axisValues = new double[6];
    private final boolean[] buttonValues = new boolean[BUTTON_COUNT];
    private final int[] povs = new int[1];

    public void updateFrom(Gamepad gamepad) {
        if (gamepad == null) {
            for (int i = 0; i < axisValues.length; i++) {
                axisValues[i] = 0.0;
            }
            for (int i = 0; i < buttonValues.length; i++) {
                buttonValues[i] = false;
            }
            povs[0] = -1;
            return;
        }

        // Axes: LeftX, LeftY, LeftTrigger, RightTrigger, RightX, RightY
        axisValues[0] = gamepad.left_stick_x;
        axisValues[1] = gamepad.left_stick_y;
        axisValues[2] = gamepad.left_trigger;
        axisValues[3] = gamepad.right_trigger;
        axisValues[4] = gamepad.right_stick_x;
        axisValues[5] = gamepad.right_stick_y;

        // Buttons (12)
        // 0..9 are the typical Xbox-style set; the last two are FTC extras.
        buttonValues[0] = gamepad.a;
        buttonValues[1] = gamepad.b;
        buttonValues[2] = gamepad.x;
        buttonValues[3] = gamepad.y;
        buttonValues[4] = gamepad.left_bumper;
        buttonValues[5] = gamepad.right_bumper;
        buttonValues[6] = gamepad.back;
        buttonValues[7] = gamepad.start;
        buttonValues[8] = gamepad.left_stick_button;
        buttonValues[9] = gamepad.right_stick_button;
        buttonValues[10] = gamepad.guide;
        buttonValues[11] = gamepad.touchpad;

        // POVs (dpad)
        povs[0] =
                gamepad.dpad_up ? 0 :
                gamepad.dpad_right ? 90 :
                gamepad.dpad_down ? 180 :
                gamepad.dpad_left ? 270 :
                -1;
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
        double[] axes = table.get("AxisValues", new double[axisValues.length]);
        int axesLen = Math.min(axes.length, axisValues.length);
        for (int i = 0; i < axesLen; i++) {
            axisValues[i] = axes[i];
        }

        boolean[] buttons = table.get("ButtonValues", new boolean[buttonValues.length]);
        int buttonsLen = Math.min(buttons.length, buttonValues.length);
        for (int i = 0; i < buttonsLen; i++) {
            buttonValues[i] = buttons[i];
        }

        int[] pov = table.get("POVs", new int[povs.length]);
        if (pov.length > 0) {
            povs[0] = pov[0];
        }
    }
}
