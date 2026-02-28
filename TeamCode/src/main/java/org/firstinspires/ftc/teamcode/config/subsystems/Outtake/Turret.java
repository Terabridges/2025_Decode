package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.ShotLeadCalculator;
import org.psilynx.psikit.core.Logger;

/**
 * Turret subsystem coordinator. Owns the three sub-components:
 * <ul>
 *   <li>{@link TurretHardware} — CRServo pair + analog encoder</li>
 *   <li>{@link TurretController} — PID + feedforward closed-loop</li>
 *   <li>{@link TurretAimController} — aim mode orchestration (vision, odo, manual)</li>
 * </ul>
 *
 * Each loop cycle:
 * <ol>
 *   <li>Hardware reads encoder, estimates velocity.</li>
 *   <li>Aim controller determines the target angle.</li>
 *   <li>Controller computes power output (PID+FF).</li>
 *   <li>Hardware sends power to CRServos.</li>
 * </ol>
 */
@Configurable
public class Turret implements Subsystem {
    private static final String LOG_PREFIX = "Turret/";

    //---------------- Backward-Compat Static Fields ----------------
    // These delegate to TurretAimController statics so existing code compiles.
    // Prefer using TurretAimController directly in new code.

    /** @deprecated Use {@link TurretAimController#turretForwardDeg} */
    public static double turretForwardDeg = TurretAimController.turretForwardDeg;
    /** @deprecated Use {@link TurretAimController#visionDirection} */
    public static double visionDirection = TurretAimController.visionDirection;
    /** @deprecated No longer used with CRServo controller. Writes are ignored. */
    public static double turretVelocity = 0.0;

    //---------------- Sub-Components ----------------
    public final TurretHardware hardware;
    public final TurretController controller;
    public final TurretAimController aimController;

    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        hardware = new TurretHardware(map);
        controller = new TurretController();
        aimController = new TurretAimController(controller);
    }

    //---------------- Convenience API ----------------

    /**
     * Set a target angle via trapezoidal motion profile.
     * The controller generates a smooth trajectory from the current position.
     */
    public void setTargetAngle(double targetDeg) {
        aimController.setPosition(getCurrentDegrees(), targetDeg);
    }

    /**
     * Set a target angle immediately (no profile, PID-only).
     * Used for continuous tracking from vision or odo.
     */
    public void setTargetAngleImmediate(double targetDeg) {
        controller.setTargetAngleImmediate(targetDeg);
    }

    /** Toggle between manual and auto aim lock. */
    public void toggleAimLock() {
        aimController.toggleAimLock(getCurrentDegrees());
    }

    /** Enable or disable aim lock. */
    public void setAimLockEnabled(boolean enabled) {
        if (enabled) {
            aimController.setAuto();
        } else {
            aimController.setManual(getCurrentDegrees());
        }
    }

    /** True when any aim-lock mode is active (AUTO, VISION_TRACK, ODO_TRACK). */
    public boolean isAimLockEnabled() {
        return aimController.isAimLockEnabled();
    }

    /** Set the manual nudge velocity in deg/sec. 0 = stop. */
    public void setManualVelocity(double velocityDegPerSec) {
        aimController.setManualVelocity(velocityDegPerSec);
    }

    /** Current turret position in degrees, from the encoder. */
    public double getCurrentDegrees() {
        return hardware.getPositionDeg();
    }

    /** Current turret angular velocity in deg/sec. */
    public double getVelocityDegPerSec() {
        return hardware.getVelocityDegPerSec();
    }

    /** True when the controller reports on-target (position + velocity thresholds). */
    public boolean isOnTarget() {
        return controller.isOnTarget(getCurrentDegrees(), getVelocityDegPerSec());
    }

    /** True when vision lock is active and tx error is within tolerance. */
    public boolean isVisionOnTarget(Vision vision, double toleranceDeg) {
        return aimController.isVisionOnTarget(vision, toleranceDeg, getCurrentDegrees());
    }

    /** Chassis yaw correction power when turret is at limit during vision lock. */
    public double getChassisYawCorrection() {
        return aimController.getChassisYawCorrection();
    }

    /** Enable/disable shot lead compensation. */
    public void setShotLeadEnabled(boolean enabled) {
        TurretAimController.shotLeadEnabled = enabled;
    }

    /** Get the shot lead calculator for external configuration. */
    public ShotLeadCalculator getShotLeadCalculator() {
        return aimController.getShotLeadCalculator();
    }

    // Aim mode delegates
    public TurretAimController.AimMode getAimMode() { return aimController.getAimMode(); }
    public TurretAimController.LockSource getActiveLockSource() { return aimController.getActiveLockSource(); }
    public TurretAimController.AimTarget getAimTarget() { return aimController.getAimTarget(); }
    public void setAimTargetGoal() { aimController.setAimTargetGoal(); }
    public void setAimTargetObelisk() { aimController.setAimTargetObelisk(); }
    public boolean isInLaunchZone() { return aimController.isInLaunchZone(); }
    public boolean isTurretWrapEnabled() { return aimController.isTurretWrapEnabled(); }

    // Odo goal heading (for telemetry / diagnostics)
    public double getOdoGoalDesiredHeadingDeg(Vision vision) {
        return aimController.getOdoGoalDesiredHeadingDeg(vision);
    }
    public double getOdoGoalHeadingDeltaDeg(Vision vision) {
        return aimController.getOdoGoalHeadingDeltaDeg(vision, getCurrentDegrees());
    }

    // Raw encoder access (for calibration OpModes)
    public double getEncoderRawDeg() { return hardware.getEncoderRawDeg(); }
    public double getEncoderVoltage() { return hardware.getEncoderVoltage(); }

    //---------------- Backward Compatibility ----------------
    // These delegate to the new API so existing test OpModes continue to compile.

    /** @deprecated Use {@link #setTargetAngle(double)} instead. */
    public void setTurretDegree(double deg) {
        setTargetAngle(deg);
    }

    /** @deprecated Use {@link #getEncoderRawDeg()} instead. */
    public double getEncoderDegrees() {
        return hardware.getEncoderRawDeg();
    }

    /** @deprecated Use {@link #getCurrentDegrees()} instead. */
    public double getMappedEncoderTurretDegrees() {
        return getCurrentDegrees();
    }

    /** @deprecated Compute error directly: {@code wrapSigned(target - getCurrentDegrees())} */
    public double getMappedEncoderErrorDeg(double targetDeg) {
        return TurretHardware.wrapSigned(getCurrentDegrees() - targetDeg);
    }

    /**
     * Send raw power to the CRServos. For calibration OpModes that bypass the controller.
     * @deprecated Use {@link TurretHardware#setPower(double)} via {@code hardware} directly.
     */
    public void setTurretPos(double normalizedPos) {
        // Old API sent a 0-1 servo position; now we reinterpret as raw power [-1, 1].
        // Callers using this for calibration should switch to hardware.setPower().
        double power = (normalizedPos - 0.5) * 2.0;
        hardware.setPower(power);
    }

    /** @deprecated Use {@link TurretAimController#setOdoTrack()} via aim controller. */
    public void aimAtObeliskWithOdometry() {
        aimController.setOdoTrack();
    }

    /**
     * Update the aim lock system. Called from Outtake.update() with the Vision subsystem.
     *
     * @param vision     Vision subsystem for target tracking
     * @param flywheelRPM current flywheel RPM for shot lead calculation
     */
    public void updateAimLock(Vision vision, double flywheelRPM) {
        aimController.update(vision, getCurrentDegrees(), flywheelRPM);
    }

    /** Backward-compatible overload (no shot lead). */
    public void updateAimLock(Vision vision) {
        updateAimLock(vision, 0.0);
    }

    //---------------- Interface Methods ----------------

    @Override
    public void toInit() {
        hardware.update(); // seed initial encoder reading
        controller.reset();
        aimController.setManual(getCurrentDegrees());

        Logger.recordOutput(LOG_PREFIX + "InitialPositionDeg", getCurrentDegrees());
    }

    @Override
    public void update() {
        // 1. Read hardware (encoder + velocity estimation)
        hardware.update();

        double currentDeg = getCurrentDegrees();
        double currentVel = getVelocityDegPerSec();

        // 2. Controller computes power
        double power = controller.update(currentDeg, currentVel);

        // 3. Send power to CRServos
        hardware.setPower(power);

        // 4. Top-level logging
        Logger.recordOutput(LOG_PREFIX + "PositionDeg", currentDeg);
        Logger.recordOutput(LOG_PREFIX + "VelocityDegSec", currentVel);
        Logger.recordOutput(LOG_PREFIX + "Power", power);
        Logger.recordOutput(LOG_PREFIX + "OnTarget", isOnTarget());
    }
}
