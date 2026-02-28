package org.firstinspires.ftc.teamcode.config.utility;

/**
 * Generates a trapezoidal velocity motion profile for point-to-point turret moves.
 *
 * Three phases: acceleration → cruise → deceleration.
 * Handles "triangular" profiles where the distance is too short to reach max velocity.
 * All units are in degrees and seconds.
 */
public class TrapezoidalMotionProfile {

    //---------------- Constraints ----------------
    private double maxVelocity;     // deg/sec
    private double maxAcceleration; // deg/sec²

    //---------------- Profile State ----------------
    private double startPos;
    private double endPos;
    private double distance;        // signed travel distance
    private double direction;       // +1 or -1
    private double absDistance;

    // Phase durations
    private double accelTime;
    private double cruiseTime;
    private double decelTime;
    private double totalTime;

    // Phase velocities
    private double peakVelocity;    // may be < maxVelocity for triangular profiles

    private boolean isGenerated = false;

    //---------------- Constructor ----------------

    /**
     * Create a profile generator with given motion constraints.
     *
     * @param maxVelocity     maximum angular velocity in deg/sec (positive)
     * @param maxAcceleration maximum angular acceleration in deg/sec² (positive)
     */
    public TrapezoidalMotionProfile(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAcceleration = Math.abs(maxAcceleration);
    }

    //---------------- Profile Generation ----------------

    /**
     * Generate a trapezoidal profile from startPos to endPos, starting from rest.
     * Call this once when a new target is set.
     *
     * @param startPos start position in degrees
     * @param endPos   end position in degrees
     */
    public void generate(double startPos, double endPos) {
        generate(startPos, endPos, 0.0);
    }

    /**
     * Generate a trapezoidal profile from startPos to endPos with an initial velocity.
     *
     * @param startPos start position in degrees
     * @param endPos   end position in degrees
     * @param startVel initial velocity in deg/sec (signed; positive = toward endPos if endPos > startPos)
     */
    public void generate(double startPos, double endPos, double startVel) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.distance = endPos - startPos;
        this.direction = Math.signum(distance);
        this.absDistance = Math.abs(distance);

        if (absDistance < 1e-6) {
            // Already at target
            accelTime = 0;
            cruiseTime = 0;
            decelTime = 0;
            totalTime = 0;
            peakVelocity = 0;
            isGenerated = true;
            return;
        }

        // Check if we can reach maxVelocity
        // Distance to accelerate from 0 to maxVelocity = v²/(2a)
        double accelDist = (maxVelocity * maxVelocity) / (2.0 * maxAcceleration);
        double decelDist = accelDist; // symmetric

        if (accelDist + decelDist <= absDistance) {
            // Full trapezoidal profile: accel + cruise + decel
            peakVelocity = maxVelocity;
            accelTime = maxVelocity / maxAcceleration;
            decelTime = accelTime;
            double cruiseDist = absDistance - accelDist - decelDist;
            cruiseTime = cruiseDist / maxVelocity;
        } else {
            // Triangular profile: can't reach maxVelocity
            // Peak velocity = sqrt(absDistance * maxAcceleration)
            peakVelocity = Math.sqrt(absDistance * maxAcceleration);
            accelTime = peakVelocity / maxAcceleration;
            decelTime = accelTime;
            cruiseTime = 0;
        }

        totalTime = accelTime + cruiseTime + decelTime;
        isGenerated = true;
    }

    //---------------- State Query ----------------

    /**
     * Get the desired motion state at a given elapsed time since profile start.
     *
     * @param elapsedSec time since profile generation
     * @return MotionState with position, velocity, and acceleration
     */
    public MotionState getState(double elapsedSec) {
        if (!isGenerated || totalTime <= 0) {
            return new MotionState(endPos, 0, 0);
        }

        double t = Math.max(0, Math.min(elapsedSec, totalTime));

        double pos, vel, accel;

        if (t <= accelTime) {
            // Acceleration phase
            accel = maxAcceleration * direction;
            vel = maxAcceleration * t * direction;
            pos = startPos + 0.5 * maxAcceleration * t * t * direction;
        } else if (t <= accelTime + cruiseTime) {
            // Cruise phase
            double tc = t - accelTime;
            accel = 0;
            vel = peakVelocity * direction;
            double accelPos = 0.5 * maxAcceleration * accelTime * accelTime;
            pos = startPos + (accelPos + peakVelocity * tc) * direction;
        } else {
            // Deceleration phase
            double td = t - accelTime - cruiseTime;
            accel = -maxAcceleration * direction;
            vel = (peakVelocity - maxAcceleration * td) * direction;
            double accelPos = 0.5 * maxAcceleration * accelTime * accelTime;
            double cruisePos = peakVelocity * cruiseTime;
            double decelPos = peakVelocity * td - 0.5 * maxAcceleration * td * td;
            pos = startPos + (accelPos + cruisePos + decelPos) * direction;
        }

        // Clamp to endPos if past total time
        if (elapsedSec >= totalTime) {
            return new MotionState(endPos, 0, 0);
        }

        return new MotionState(pos, vel, accel);
    }

    /** True when elapsed time exceeds the profile duration. */
    public boolean isFinished(double elapsedSec) {
        return !isGenerated || elapsedSec >= totalTime;
    }

    /** Total profile duration in seconds. */
    public double getTotalTime() {
        return totalTime;
    }

    /** Whether a profile has been generated. */
    public boolean isGenerated() {
        return isGenerated;
    }

    /** Update motion constraints for live tuning. */
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAcceleration = Math.abs(maxAcceleration);
    }

    /** Peak velocity the profile reaches (may be less than maxVelocity for short moves). */
    public double getPeakVelocity() {
        return peakVelocity;
    }

    /** The start position of the current profile. */
    public double getStartPos() {
        return startPos;
    }

    /** The end position (target) of the current profile. */
    public double getEndPos() {
        return endPos;
    }

    //---------------- Motion State ----------------

    /**
     * Represents the desired position, velocity, and acceleration at a point in time.
     */
    public static class MotionState {
        public final double position;     // degrees
        public final double velocity;     // deg/sec
        public final double acceleration; // deg/sec²

        public MotionState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }
}
