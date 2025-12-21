package org.firstinspires.ftc.teamcode.config.control.ftc;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.control.GamepadView;

/** Wraps an FTC SDK {@link Gamepad} as a desktop-safe {@link GamepadView}. */
public final class FtcGamepadView implements GamepadView {

    private final Gamepad gamepad;

    public FtcGamepadView(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    @Override
    public double leftStickX() {
        return gamepad.left_stick_x;
    }

    @Override
    public double leftStickY() {
        return gamepad.left_stick_y;
    }

    @Override
    public double rightStickX() {
        return gamepad.right_stick_x;
    }

    @Override
    public boolean dpadUp() {
        return gamepad.dpad_up;
    }

    @Override
    public boolean dpadDown() {
        return gamepad.dpad_down;
    }

    @Override
    public boolean dpadLeft() {
        return gamepad.dpad_left;
    }

    @Override
    public boolean dpadRight() {
        return gamepad.dpad_right;
    }

    @Override
    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    @Override
    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    @Override
    public double leftTrigger() {
        return gamepad.left_trigger;
    }

    @Override
    public double rightTrigger() {
        return gamepad.right_trigger;
    }

    @Override
    public boolean a() {
        return gamepad.a;
    }

    @Override
    public boolean b() {
        return gamepad.b;
    }

    @Override
    public boolean x() {
        return gamepad.x;
    }

    @Override
    public boolean y() {
        return gamepad.y;
    }

    @Override
    public boolean leftStickButton() {
        return gamepad.left_stick_button;
    }

    @Override
    public boolean rightStickButton() {
        return gamepad.right_stick_button;
    }
}
