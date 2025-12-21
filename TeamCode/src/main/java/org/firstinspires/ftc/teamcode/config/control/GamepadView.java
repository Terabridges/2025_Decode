package org.firstinspires.ftc.teamcode.config.control;

/**
 * Desktop-safe view of gamepad inputs.
 *
 * This exists so control code can run both on-robot (wrapping FTC {@code Gamepad})
 * and on desktop during replay.
 */
public interface GamepadView {
    double leftStickX();

    double leftStickY();

    double rightStickX();

    boolean dpadUp();

    boolean dpadDown();

    boolean dpadLeft();

    boolean dpadRight();

    boolean leftBumper();

    boolean rightBumper();

    double leftTrigger();

    double rightTrigger();

    boolean a();

    boolean b();

    boolean x();

    boolean y();

    boolean leftStickButton();

    boolean rightStickButton();
}
