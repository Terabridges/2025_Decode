package org.firstinspires.ftc.teamcode.config.control;

/**
 * Desktop-safe mecanum drive math extracted from {@link DriveControl}.
 */
public final class MecanumDriveMath {

    private MecanumDriveMath() {
        // Utility class
    }

    /**
     * Computes normalized wheel powers for mecanum drive.
     *
     * @param axial   forward (+) / backward (-)
     * @param lateral strafe right (+) / left (-)
     * @param yaw     rotate CCW (+) / CW (-)
     */
    public static double[] computeWheelPowers(double axial, double lateral, double yaw) {
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        return new double[] {leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }

    /**
     * Matches {@link DriveControl}'s joystick interpretation:
     * axial = -leftStickY, lateral = leftStickX, yaw = rightStickX.
     */
    public static double[] computeWheelPowersFromSticks(double leftStickX, double leftStickY, double rightStickX) {
        return computeWheelPowers(-leftStickY, leftStickX, rightStickX);
    }
}
