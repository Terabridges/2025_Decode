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

    // Axes
    public double leftX;
    public double leftY;
    public double rightX;
    public double rightY;

    // Triggers (0..1)
    public double leftTrigger;
    public double rightTrigger;

    // Buttons
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;

    public boolean leftBumper;
    public boolean rightBumper;

    public boolean back;
    public boolean start;

    public boolean leftStick;
    public boolean rightStick;

    public void updateFrom(Gamepad gamepad) {
        if (gamepad == null) {
            leftX = 0.0;
            leftY = 0.0;
            rightX = 0.0;
            rightY = 0.0;
            leftTrigger = 0.0;
            rightTrigger = 0.0;
            a = b = x = y = false;
            leftBumper = rightBumper = false;
            back = start = false;
            leftStick = rightStick = false;
            return;
        }

        leftX = gamepad.left_stick_x;
        leftY = gamepad.left_stick_y;
        rightX = gamepad.right_stick_x;
        rightY = gamepad.right_stick_y;

        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;

        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;

        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;

        back = gamepad.back;
        start = gamepad.start;

        leftStick = gamepad.left_stick_button;
        rightStick = gamepad.right_stick_button;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("LeftX", leftX);
        table.put("LeftY", leftY);
        table.put("RightX", rightX);
        table.put("RightY", rightY);

        table.put("LeftTrigger", leftTrigger);
        table.put("RightTrigger", rightTrigger);

        table.put("A", a);
        table.put("B", b);
        table.put("X", x);
        table.put("Y", y);

        table.put("LeftBumper", leftBumper);
        table.put("RightBumper", rightBumper);

        table.put("Back", back);
        table.put("Start", start);

        table.put("LeftStick", leftStick);
        table.put("RightStick", rightStick);
    }

    @Override
    public void fromLog(LogTable table) {
        leftX = table.get("LeftX", 0.0);
        leftY = table.get("LeftY", 0.0);
        rightX = table.get("RightX", 0.0);
        rightY = table.get("RightY", 0.0);

        leftTrigger = table.get("LeftTrigger", 0.0);
        rightTrigger = table.get("RightTrigger", 0.0);

        a = table.get("A", false);
        b = table.get("B", false);
        x = table.get("X", false);
        y = table.get("Y", false);

        leftBumper = table.get("LeftBumper", false);
        rightBumper = table.get("RightBumper", false);

        back = table.get("Back", false);
        start = table.get("Start", false);

        leftStick = table.get("LeftStick", false);
        rightStick = table.get("RightStick", false);
    }
}
