package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.utility.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.config.utility.TrapezoidalMotionProfile.MotionState;
import org.psilynx.psikit.core.Logger;

/**
 * Closed-loop PID + feedforward controller for the turret CRServo.
 *
 * <h3>Control Modes</h3>
 * <ul>
 *   <li><b>PROFILE</b> — follows a {@link TrapezoidalMotionProfile} to reach a target angle.
 *       Switches to HOLD automatically when the profile completes and position is on-target.</li>
 *   <li><b>HOLD</b> — holds the current target with PID only (no profile).</li>
 *   <li><b>DIRECT</b> — accepts an external power command. Used for manual/joystick and auto-tuning.</li>
 * </ul>
 *
 * <h3>Feedforward</h3>
 * <ul>
 *   <li><b>kS</b> — static friction compensation: {@code kS * signum(desiredVelocity)}
 *       whenever velocity setpoint is non-zero.</li>
 *   <li><b>kV</b> — velocity feedforward: {@code kV * desiredVelocity}.</li>
 *   <li><b>kA</b> — acceleration feedforward: {@code kA * desiredAcceleration}.</li>
 * </ul>
 */
@Configurable
public class TurretController {
    private static final String LOG_PREFIX = "Turret/Controller/";

    public enum ControlMode {
        PROFILE,
        HOLD,
        DIRECT
    }

    //---------------- Feedforward Gains ----------------
    /** Stiction power for CW (positive) direction — measured via TurretStictionCharacterizer. */
    public static double kS_CW = 0.084;
    /** Stiction power for CCW (negative) direction — measured via TurretStictionCharacterizer. */
    public static double kS_CCW = 0.072;
    /** Velocity feedforward (power per deg/sec) — measured via TurretVelocityCharacterizer. */
    public static double kV = 0.00068;
    /** Acceleration feedforward (power per deg/sec²). */
    public static double kA = 0.0001;

    //---------------- PID Gains ----------------
    // CRServo = power→velocity (not torque), so gains must be VERY small.
    // At kV=0.00068, 1° error should produce only ~0.002 power (→ ~3°/s approach).
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0005;
    /** Integral only accumulates when |error| < iZone (degrees). 0 = always. */
    public static double iZone = 10.0;
    /** Maximum integral accumulation (prevents windup). */
    public static double maxIntegral = 0.3;
    /** Low-pass filter coefficient for derivative (0–1; 1 = no filter). */
    public static double dAlpha = 0.4;

    //---------------- Motion Profile Constraints ----------------
    public static double maxProfileVelocity = 150.0;     // deg/sec
    public static double maxProfileAcceleration = 300.0;  // deg/sec²

    //---------------- Output Limits ----------------
    /**
     * Maximum absolute power output from the controller (0–1).
     * Prevents full-power commands that cause violent overshoot on CRServos.
     * Should be set just above what the turret needs for the desired profile velocity.
     */
    public static double maxOutputPower = 0.30;

    //---------------- On-Target Thresholds ----------------
    /** Position must be within this tolerance (degrees) to be considered on-target. */
    public static double onTargetPositionDeg = 2.0;
    /** Velocity must be below this threshold (deg/sec) to be considered settled. */
    public static double onTargetVelocityDeg = 10.0;

    //---------------- State ----------------
    private ControlMode mode = ControlMode.HOLD;
    private double targetDeg = 180.0;
    private TrapezoidalMotionProfile profile;
    private final ElapsedTime profileTimer = new ElapsedTime();

    // PID state
    private double integral = 0.0;
    private double prevError = 0.0;
    private double filteredDerivative = 0.0;
    private double prevTimeSec = -1.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Last outputs (for logging)
    private double lastOutput = 0.0;
    private double lastError = 0.0;
    private double lastFeedforward = 0.0;
    private double lastPidOutput = 0.0;
    private MotionState lastProfileState = null;

    //---------------- Constructor ----------------
    public TurretController() {
        profile = new TrapezoidalMotionProfile(maxProfileVelocity, maxProfileAcceleration);
        loopTimer.reset();
    }

    //---------------- Public Methods ----------------

    /**
     * Command the turret to move to a target angle using a trapezoidal motion profile.
     * The profile starts from the current position and generates a smooth trajectory.
     *
     * @param currentDeg current turret position in degrees
     * @param target     desired turret angle in degrees
     */
    public void setTargetAngle(double currentDeg, double target) {
        targetDeg = target;
        profile.setConstraints(maxProfileVelocity, maxProfileAcceleration);
        profile.generate(currentDeg, target);
        profileTimer.reset();
        mode = ControlMode.PROFILE;
        // Don't reset integral — allows smooth transition
    }

    /**
     * Set the target angle without generating a profile. Used for vision/odo tracking
     * where the target changes every frame. The controller uses PID-only (HOLD mode).
     *
     * @param target desired turret angle in degrees
     */
    public void setTargetAngleImmediate(double target) {
        targetDeg = target;
        if (mode == ControlMode.PROFILE && profile.isGenerated()) {
            // If we were profiling, switch to hold to avoid stale profile state
            mode = ControlMode.HOLD;
        } else if (mode == ControlMode.DIRECT) {
            mode = ControlMode.HOLD;
        }
    }

    /**
     * Switch to DIRECT mode with a specific power. Used for manual joystick or auto-tuning.
     *
     * @param power motor power in [-1.0, 1.0]
     */
    public void setDirectPower(double power) {
        mode = ControlMode.DIRECT;
        lastOutput = Math.max(-1.0, Math.min(1.0, power));
    }

    /**
     * Compute the power output for this loop. Must be called once per loop cycle.
     *
     * @param currentDeg        current turret position in degrees
     * @param currentVelDegPerSec current turret velocity in deg/sec
     * @return power in [-1.0, 1.0]
     */
    public double update(double currentDeg, double currentVelDegPerSec) {
        double nowSec = loopTimer.seconds();
        double dtSec = (prevTimeSec < 0) ? 0.02 : (nowSec - prevTimeSec);
        prevTimeSec = nowSec;
        dtSec = Math.max(dtSec, 0.001); // safety floor

        double output;

        if (mode == ControlMode.DIRECT) {
            output = lastOutput;
        } else if (mode == ControlMode.PROFILE) {
            output = updateProfile(currentDeg, currentVelDegPerSec, dtSec);
        } else {
            // HOLD mode — PID only, no feedforward velocity/accel
            output = updateHold(currentDeg, dtSec);
        }

        // Clamp to maxOutputPower — CRServos have too much momentum at full power
        output = Math.max(-maxOutputPower, Math.min(maxOutputPower, output));
        lastOutput = output;

        // Logging
        Logger.recordOutput(LOG_PREFIX + "Mode", mode);
        Logger.recordOutput(LOG_PREFIX + "TargetDeg", targetDeg);
        Logger.recordOutput(LOG_PREFIX + "ErrorDeg", lastError);
        Logger.recordOutput(LOG_PREFIX + "Output", lastOutput);
        Logger.recordOutput(LOG_PREFIX + "Feedforward", lastFeedforward);
        Logger.recordOutput(LOG_PREFIX + "PID", lastPidOutput);
        Logger.recordOutput(LOG_PREFIX + "Integral", integral);
        Logger.recordOutput(LOG_PREFIX + "FilteredDeriv", filteredDerivative);
        Logger.recordOutput(LOG_PREFIX + "OnTarget", isOnTarget(currentDeg, currentVelDegPerSec));

        if (lastProfileState != null) {
            Logger.recordOutput(LOG_PREFIX + "Profile/DesiredPosDeg", lastProfileState.position);
            Logger.recordOutput(LOG_PREFIX + "Profile/DesiredVelDegSec", lastProfileState.velocity);
            Logger.recordOutput(LOG_PREFIX + "Profile/DesiredAccelDegSec2", lastProfileState.acceleration);
        }

        return output;
    }

    /**
     * Returns true when the turret is at the target position and velocity is near zero.
     */
    public boolean isOnTarget(double currentDeg, double currentVelDegPerSec) {
        double error = wrapSigned(targetDeg - currentDeg);
        return Math.abs(error) <= onTargetPositionDeg
                && Math.abs(currentVelDegPerSec) <= onTargetVelocityDeg;
    }

    /** Reset PID state (integral, derivative filter). */
    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        filteredDerivative = 0.0;
        prevTimeSec = -1.0;
        lastProfileState = null;
        lastFeedforward = 0.0;
        lastPidOutput = 0.0;
    }

    /** Current control mode. */
    public ControlMode getMode() {
        return mode;
    }

    /** Current target in degrees. */
    public double getTargetDeg() {
        return targetDeg;
    }

    /** Get the underlying profile (for external inspection during tuning). */
    public TrapezoidalMotionProfile getProfile() {
        return profile;
    }

    /** Elapsed time since last profile was generated. */
    public double getProfileElapsedSec() {
        return profileTimer.seconds();
    }

    //---------------- Internal ----------------

    private double updateProfile(double currentDeg, double currentVelDegPerSec, double dtSec) {
        double elapsed = profileTimer.seconds();
        MotionState desired = profile.getState(elapsed);
        lastProfileState = desired;

        // If profile is finished and we're on target, switch to HOLD
        if (profile.isFinished(elapsed) && isOnTarget(currentDeg, currentVelDegPerSec)) {
            mode = ControlMode.HOLD;
            lastProfileState = null;
            return updateHold(currentDeg, dtSec);
        }

        // Feedforward from profile (direction-specific stiction)
        double ff = 0.0;
        double error = wrapSigned(desired.position - currentDeg);

        if (Math.abs(desired.velocity) > 0.01) {
            // Profile is actively moving—use desired velocity direction for stiction
            double kS = (desired.velocity > 0) ? kS_CW : kS_CCW;
            ff += kS * Math.signum(desired.velocity);
        } else if (Math.abs(error) > onTargetPositionDeg * 0.5) {
            // Profile finished but not on target—use error direction for stiction
            // (Without this, PID alone produces ~0.06 which is below stiction ~0.08)
            double kS = (error > 0) ? kS_CW : kS_CCW;
            ff += kS * Math.signum(error);
        }
        ff += kV * desired.velocity;
        ff += kA * desired.acceleration;
        lastFeedforward = ff;

        // PID on tracking error (desired position from profile vs actual)
        double pid = computePID(error, dtSec);
        lastPidOutput = pid;

        return ff + pid;
    }

    private double updateHold(double currentDeg, double dtSec) {
        lastProfileState = null;
        lastFeedforward = 0.0;

        double error = wrapSigned(targetDeg - currentDeg);
        double pid = computePID(error, dtSec);
        lastPidOutput = pid;

        // Add stiction compensation only when error is meaningful (direction-specific)
        double ff = 0.0;
        if (Math.abs(error) > onTargetPositionDeg * 0.5) {
            double kS = (error > 0) ? kS_CW : kS_CCW;
            ff = kS * Math.signum(error);
        }
        lastFeedforward = ff;

        return ff + pid;
    }

    private double computePID(double error, double dtSec) {
        lastError = error;

        // Proportional
        double p = kP * error;

        // Integral with zone and anti-windup
        if (iZone <= 0 || Math.abs(error) < iZone) {
            integral += error * dtSec;
        } else {
            integral = 0.0; // outside iZone, reset
        }
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double i = kI * integral;

        // Derivative with low-pass filtering
        double rawDerivative = (error - prevError) / dtSec;
        double alpha = Math.max(0.0, Math.min(1.0, dAlpha));
        filteredDerivative = (alpha * rawDerivative) + ((1.0 - alpha) * filteredDerivative);
        double d = kD * filteredDerivative;

        prevError = error;
        return p + i + d;
    }

    /** Wrap angle to [-180, 180). */
    private static double wrapSigned(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
