package org.firstinspires.ftc.teamcode.config.utility;

import com.bylazar.configurables.annotations.Configurable;

import org.psilynx.psikit.core.Logger;

/**
 * Computes a turret aim offset to compensate for chassis velocity during shooting.
 *
 * <h3>Physics</h3>
 * <ol>
 *   <li>Estimate time-of-flight: {@code distance / ballExitVelocity}</li>
 *   <li>During TOF, chassis moves: {@code dx = vx * tof, dy = vy * tof}</li>
 *   <li>Compute shifted aim point: aim where the target appears from the robot's
 *       <em>future</em> position (or equivalently, shift the target by the negative
 *       of the chassis translation).</li>
 *   <li>Convert the shifted aim point to a turret bearing offset in degrees.</li>
 * </ol>
 *
 * This class is a standalone utility — it does not depend on any subsystem and can be
 * used from any OpMode or aim controller.
 */
@Configurable
public class ShotLeadCalculator {
    private static final String LOG_PREFIX = "Turret/ShotLead/";

    //---------------- Configuration ----------------
    /** Blend factor: 0.0 = no lead, 1.0 = full physics lead. Useful for gradual tuning. */
    public static double leadGain = 1.0;
    /** Flywheel wheel diameter in inches (for RPM → linear velocity conversion). */
    public static double flywheelDiameterIn = 4.0;
    /** Fraction of flywheel surface speed that transfers to the ball (slip/compression losses). */
    public static double launchEfficiency = 0.7;
    /** Minimum distance (inches) below which shot lead is not applied (too close = negligible). */
    public static double minDistanceIn = 24.0;
    /** Maximum lead offset in degrees (safety clamp). */
    public static double maxLeadOffsetDeg = 15.0;

    //---------------- Last Result (for logging) ----------------
    private double lastLeadOffsetDeg = 0.0;
    private double lastTimeOfFlightSec = 0.0;
    private double lastBallSpeedInPerSec = 0.0;

    //---------------- Core Method ----------------

    /**
     * Calculate the turret aim offset needed to compensate for chassis motion.
     *
     * @param chassisVxInPerSec field-frame X velocity of the robot (inches/sec)
     * @param chassisVyInPerSec field-frame Y velocity of the robot (inches/sec)
     * @param robotHeadingRad   robot heading in radians
     * @param turretForwardDeg  turret angle that corresponds to the robot's forward direction
     * @param turretCurrentDeg  current (or desired) turret angle in degrees
     * @param distanceToTargetIn distance from robot to target in inches
     * @param flywheelRPM       current flywheel speed in RPM
     * @return signed offset in degrees to add to the turret target angle
     */
    public double calculateLeadOffsetDeg(
            double chassisVxInPerSec, double chassisVyInPerSec,
            double robotHeadingRad,
            double turretForwardDeg, double turretCurrentDeg,
            double distanceToTargetIn, double flywheelRPM) {

        if (distanceToTargetIn < minDistanceIn || leadGain <= 0.0) {
            lastLeadOffsetDeg = 0.0;
            lastTimeOfFlightSec = 0.0;
            log();
            return 0.0;
        }

        // Ball exit velocity from flywheel RPM
        double ballSpeedInPerSec = ballExitVelocityFromRPM(flywheelRPM);
        lastBallSpeedInPerSec = ballSpeedInPerSec;

        if (ballSpeedInPerSec < 1.0) {
            lastLeadOffsetDeg = 0.0;
            lastTimeOfFlightSec = 0.0;
            log();
            return 0.0;
        }

        // Time of flight (seconds)
        double tof = distanceToTargetIn / ballSpeedInPerSec;
        lastTimeOfFlightSec = tof;

        // Chassis displacement during TOF (field frame)
        double dxField = chassisVxInPerSec * tof;
        double dyField = chassisVyInPerSec * tof;

        // Convert chassis displacement to robot frame
        double cosH = Math.cos(-robotHeadingRad);
        double sinH = Math.sin(-robotHeadingRad);
        double dxRobot = dxField * cosH - dyField * sinH;
        double dyRobot = dxField * sinH + dyField * cosH;

        // The turret fires along its current heading relative to the robot.
        // The turret's forward direction in the robot frame is (turretCurrentDeg - turretForwardDeg) from
        // the robot's forward axis. We need the angular offset caused by the robot's lateral motion
        // perpendicular to the shot direction.

        double turretRelativeRad = Math.toRadians(turretCurrentDeg - turretForwardDeg);

        // Project chassis motion onto the axis perpendicular to the shot direction
        // Shot direction unit vector: (sin(turretRelative), cos(turretRelative))
        // Perpendicular (leftward): (-cos(turretRelative), sin(turretRelative))
        double perpDisplacement = -dxRobot * Math.cos(turretRelativeRad)
                + dyRobot * Math.sin(turretRelativeRad);

        // Angular offset = atan2(perpDisplacement, distance)
        double offsetRad = Math.atan2(perpDisplacement, distanceToTargetIn);
        double offsetDeg = Math.toDegrees(offsetRad) * leadGain;

        // Clamp for safety
        offsetDeg = Math.max(-maxLeadOffsetDeg, Math.min(maxLeadOffsetDeg, offsetDeg));

        lastLeadOffsetDeg = offsetDeg;
        log();
        return offsetDeg;
    }

    //---------------- Utility ----------------

    /**
     * Convert flywheel RPM to ball exit velocity in inches/second.
     * Accounts for wheel diameter and launch efficiency.
     */
    public double ballExitVelocityFromRPM(double rpm) {
        // circumference (in) * revs/sec * efficiency
        double circumferenceIn = Math.PI * flywheelDiameterIn;
        double revsPerSec = rpm / 60.0;
        return circumferenceIn * revsPerSec * launchEfficiency;
    }

    /** Last computed lead offset in degrees. */
    public double getLastLeadOffsetDeg() {
        return lastLeadOffsetDeg;
    }

    /** Last computed time-of-flight in seconds. */
    public double getLastTimeOfFlightSec() {
        return lastTimeOfFlightSec;
    }

    /** Last computed ball exit speed in inches/sec. */
    public double getLastBallSpeedInPerSec() {
        return lastBallSpeedInPerSec;
    }

    private void log() {
        Logger.recordOutput(LOG_PREFIX + "LeadOffsetDeg", lastLeadOffsetDeg);
        Logger.recordOutput(LOG_PREFIX + "TimeOfFlightSec", lastTimeOfFlightSec);
        Logger.recordOutput(LOG_PREFIX + "BallSpeedInPerSec", lastBallSpeedInPerSec);
        Logger.recordOutput(LOG_PREFIX + "LeadGain", leadGain);
    }
}
