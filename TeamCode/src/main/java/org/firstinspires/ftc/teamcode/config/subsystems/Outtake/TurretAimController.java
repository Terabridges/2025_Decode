package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.ShotLeadCalculator;
import org.psilynx.psikit.core.Logger;

/**
 * Aim-mode orchestrator for the turret. Decides <em>what angle</em> the turret should target
 * each loop, then delegates to {@link TurretController} for <em>how</em> to get there.
 *
 * <h3>Aim Modes</h3>
 * <ul>
 *   <li>{@link AimMode#MANUAL} — holds last commanded angle; accepts manual nudge velocity.</li>
 *   <li>{@link AimMode#POSITION} — profiled move to a specific angle.</li>
 *   <li>{@link AimMode#VISION_TRACK} — closed-loop on Limelight tx.</li>
 *   <li>{@link AimMode#ODO_TRACK} — aim at a field point using follower pose.</li>
 *   <li>{@link AimMode#AUTO} — vision-preferred with odometry fallback.</li>
 * </ul>
 */
@Configurable
public class TurretAimController {
    private static final String LOG_PREFIX = "Turret/Aim/";
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    public enum AimMode {
        MANUAL,
        POSITION,
        VISION_TRACK,
        ODO_TRACK,
        AUTO
    }

    public enum LockSource {
        NONE,
        TX,
        ODO
    }

    public enum AimTarget {
        GOAL,
        OBELISK
    }

    //---------------- Vision Lock Tuning ----------------
    public static double visionKp = 0.27;
    public static double visionDeadbandDeg = 1.2;
    public static double visionMaxStepDeg = 1.2;
    public static double visionMinStepDeg = 0.16;
    public static double visionTxAlpha = 0.3;
    public static double cameraLateralOffsetIn = 0.0;
    public static double visionDirection = 1.0;
    public static double blueVisionCloseBiasDeg = 0;
    public static double blueVisionFarBiasDeg = 2.5;
    public static double redVisionCloseBiasDeg = 0;
    public static double redVisionFarBiasDeg = 1.5;
    public static double visionDistanceSplitIn = 110.0;

    //---------------- Odometry Lock Tuning ----------------
    public static double odoKp = 0.95;
    public static double odoDeadbandDeg = 1.0;
    public static double odoMaxStepDeg = 4.5;
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 144.0;
    public static double redGoalX = 144.0;
    public static double redGoalY = 144.0;
    public static double obeliskX = 72.0;
    public static double obeliskY = 144.0;

    //---------------- Turret Geometry ----------------
    public static double turretForwardDeg = 163.0;
    public static double turretMinDeg = 18.0;
    public static double turretMaxDeg = 330.0;

    //---------------- Edge Correction ----------------
    public static double edgeCorrectionMinScale = 0.65;
    public static double edgeCorrectionMarginDeg = 35.0;
    public static double limitAssistMarginDeg = 1.0;

    //---------------- Fallback Hysteresis ----------------
    public static double visionFallbackDelayMs = 180.0;
    public static double visionReacquireDelayMs = 140.0;
    public static double odoHoldAfterFallbackMs = 220.0;

    //---------------- Launch Zone (Wrap) ----------------
    public static boolean autoEnableWrapInLaunchZone = true;
    public static boolean wrapWithinLaunchZoneOnly = true;
    public static double goalLaunchBaseLeftX = 0.0;
    public static double goalLaunchBaseRightX = 144.0;
    public static double goalLaunchBaseY = 144.0;
    public static double goalLaunchApexX = 72.0;
    public static double goalLaunchApexY = 72.0;
    public static double audienceLaunchBaseLeftX = 48.0;
    public static double audienceLaunchBaseRightX = 96.0;
    public static double audienceLaunchBaseY = 0.0;
    public static double audienceLaunchApexX = 72.0;
    public static double audienceLaunchApexY = 24.0;

    //---------------- Manual Nudge ----------------
    public static double maxManualVelocityDegPerSec = 180.0;

    //---------------- Shot Lead ----------------
    public static boolean shotLeadEnabled = false;

    //---------------- Chassis Yaw Assist ----------------
    public static double chassisYawKp = 0.02;
    public static double chassisYawMax = 0.25;

    //---------------- State ----------------
    private AimMode aimMode = AimMode.MANUAL;
    private LockSource activeLockSource = LockSource.NONE;
    private AimTarget aimTarget = AimTarget.GOAL;
    private boolean turretWrapEnabled = false;
    private boolean inLaunchZone = false;
    private double manualTargetDeg = 180.0;
    private double manualVelocityDegPerSec = 0.0;

    // Vision EMA filter
    private boolean hasFilteredTx = false;
    private double filteredTxDeg = 0.0;

    // Fallback hysteresis timers
    private final ElapsedTime visionLostTimer = new ElapsedTime();
    private final ElapsedTime visionReacquireTimer = new ElapsedTime();
    private boolean wasVisionActive = false;
    private boolean inFallback = false;

    // Loop timing for manual nudge integration
    private double prevTimeSec = -1.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Chassis yaw assist output
    private double chassisYawCorrection = 0.0;

    // Shot lead calculator
    private final ShotLeadCalculator shotLeadCalc = new ShotLeadCalculator();

    // Reference to controller for profiled moves
    private TurretController controller;

    //---------------- Constructor ----------------
    public TurretAimController(TurretController controller) {
        this.controller = controller;
        loopTimer.reset();
        visionLostTimer.reset();
        visionReacquireTimer.reset();
    }

    //---------------- Main Update ----------------

    /**
     * Main update loop. Determines the target angle based on the current aim mode
     * and feeds it to the TurretController.
     *
     * @param vision       Vision subsystem (may be null)
     * @param currentDeg   current turret position from TurretHardware
     * @param flywheelRPM  current flywheel RPM (for shot lead)
     */
    public void update(Vision vision, double currentDeg, double flywheelRPM) {
        double nowSec = loopTimer.seconds();
        double dtSec = (prevTimeSec < 0) ? 0.02 : (nowSec - prevTimeSec);
        prevTimeSec = nowSec;
        dtSec = Math.max(dtSec, 0.001);

        refreshWrapState();
        chassisYawCorrection = 0.0;

        switch (aimMode) {
            case MANUAL:
                updateManual(currentDeg, dtSec);
                break;
            case POSITION:
                // Controller is already profiling; nothing to update here
                break;
            case VISION_TRACK:
                updateVisionTrack(vision, currentDeg, flywheelRPM);
                break;
            case ODO_TRACK:
                updateOdoTrack(currentDeg, flywheelRPM);
                break;
            case AUTO:
                updateAuto(vision, currentDeg, flywheelRPM);
                break;
        }

        // Logging
        Logger.recordOutput(LOG_PREFIX + "Mode", aimMode);
        Logger.recordOutput(LOG_PREFIX + "LockSource", activeLockSource);
        Logger.recordOutput(LOG_PREFIX + "AimTarget", aimTarget);
        Logger.recordOutput(LOG_PREFIX + "ManualTargetDeg", manualTargetDeg);
        Logger.recordOutput(LOG_PREFIX + "ManualVelocity", manualVelocityDegPerSec);
        Logger.recordOutput(LOG_PREFIX + "InLaunchZone", inLaunchZone);
        Logger.recordOutput(LOG_PREFIX + "WrapEnabled", turretWrapEnabled);
        Logger.recordOutput(LOG_PREFIX + "ChassisYawCorrection", chassisYawCorrection);
        Logger.recordOutput(LOG_PREFIX + "ShotLeadEnabled", shotLeadEnabled);
        Logger.recordOutput(LOG_PREFIX + "InFallback", inFallback);
        if (hasFilteredTx) {
            Logger.recordOutput(LOG_PREFIX + "FilteredTxDeg", filteredTxDeg);
        }
    }

    //---------------- Mode Switching ----------------

    /** Switch to MANUAL mode. The turret holds its current position. */
    public void setManual(double currentDeg) {
        aimMode = AimMode.MANUAL;
        manualTargetDeg = currentDeg;
        manualVelocityDegPerSec = 0.0;
        activeLockSource = LockSource.NONE;
        controller.setTargetAngleImmediate(currentDeg);
    }

    /** Go to a specific angle via profiled trajectory. */
    public void setPosition(double currentDeg, double targetDeg) {
        aimMode = AimMode.POSITION;
        activeLockSource = LockSource.NONE;
        controller.setTargetAngle(currentDeg, clampToSafeRange(targetDeg));
    }

    /** Track target with vision. */
    public void setVisionTrack() {
        aimMode = AimMode.VISION_TRACK;
        manualVelocityDegPerSec = 0.0;
        hasFilteredTx = false;
        activeLockSource = LockSource.TX;
    }

    /** Track a field point with odometry. */
    public void setOdoTrack() {
        aimMode = AimMode.ODO_TRACK;
        manualVelocityDegPerSec = 0.0;
        activeLockSource = LockSource.ODO;
    }

    /** Vision-preferred with odo fallback. */
    public void setAuto() {
        aimMode = AimMode.AUTO;
        manualVelocityDegPerSec = 0.0;
        hasFilteredTx = false;
        inFallback = false;
        activeLockSource = LockSource.NONE;
    }

    /** Toggle between MANUAL and AUTO modes. */
    public void toggleAimLock(double currentDeg) {
        if (aimMode == AimMode.AUTO || aimMode == AimMode.VISION_TRACK || aimMode == AimMode.ODO_TRACK) {
            setManual(currentDeg);
        } else {
            setAuto();
        }
    }

    /** Set the manual velocity for nudge control (deg/sec). 0 = stop nudging. */
    public void setManualVelocity(double velocityDegPerSec) {
        this.manualVelocityDegPerSec = velocityDegPerSec;
    }

    //---------------- Target & Mode Accessors ----------------

    public AimMode getAimMode() { return aimMode; }
    public LockSource getActiveLockSource() { return activeLockSource; }
    public AimTarget getAimTarget() { return aimTarget; }
    public void setAimTargetGoal() { aimTarget = AimTarget.GOAL; }
    public void setAimTargetObelisk() { aimTarget = AimTarget.OBELISK; }
    public boolean isAimLockEnabled() {
        return aimMode == AimMode.AUTO || aimMode == AimMode.VISION_TRACK || aimMode == AimMode.ODO_TRACK;
    }
    public boolean isInLaunchZone() { return inLaunchZone; }
    public boolean isTurretWrapEnabled() { return turretWrapEnabled; }

    /**
     * Returns the chassis yaw correction power the drive should apply to assist the turret
     * when it is at a limit. Positive = rotate CW, negative = rotate CCW.
     * Only meaningful when aim lock is active.
     */
    public double getChassisYawCorrection() { return chassisYawCorrection; }

    public ShotLeadCalculator getShotLeadCalculator() { return shotLeadCalc; }

    //---------------- Vision On-Target ----------------

    /**
     * True when vision lock is active and the turret is pointed at the target within tolerance.
     */
    public boolean isVisionOnTarget(Vision vision, double toleranceDeg, double currentDeg) {
        if (vision == null) return false;
        int requiredTagId = vision.getRequiredTagId();
        boolean hasTarget = requiredTagId >= 0
                ? vision.seesTag(requiredTagId)
                : vision.hasTarget();
        if (!hasTarget) return false;

        double tx = requiredTagId >= 0
                ? vision.getTxForTag(requiredTagId)
                : vision.getTx();
        double distanceIn = requiredTagId >= 0
                ? vision.getDistanceInchesForTag(requiredTagId)
                : vision.getDistanceInches();
        return Math.abs(computeVisionAimErrorDeg(tx, distanceIn)) <= Math.abs(toleranceDeg);
    }

    //---------------- MANUAL Mode ----------------

    private void updateManual(double currentDeg, double dtSec) {
        activeLockSource = LockSource.NONE;

        if (Math.abs(manualVelocityDegPerSec) > 0.01) {
            manualTargetDeg += manualVelocityDegPerSec * dtSec;
            manualTargetDeg = clampToSafeRange(normalizeDegrees(manualTargetDeg));
            controller.setTargetAngleImmediate(manualTargetDeg);
        }
    }

    //---------------- VISION_TRACK Mode ----------------

    private void updateVisionTrack(Vision vision, double currentDeg, double flywheelRPM) {
        activeLockSource = LockSource.TX;

        int tagId = getRequiredGoalTagId(vision);
        if (vision == null || !vision.seesTag(tagId)) {
            // No target — hold current position
            return;
        }

        double rawTx = vision.getTxForTag(tagId);
        double filteredTx = filterTx(rawTx);
        double distanceIn = vision.getDistanceInchesForTag(tagId);

        double targetDeg = computeVisionTargetDeg(filteredTx, distanceIn, currentDeg);
        targetDeg = applyShotLead(targetDeg, distanceIn, flywheelRPM, currentDeg);
        targetDeg = clampToSafeRange(normalizeDegrees(targetDeg));

        controller.setTargetAngleImmediate(targetDeg);
        computeChassisYawAssist(filteredTx, distanceIn, currentDeg);
    }

    //---------------- ODO_TRACK Mode ----------------

    private void updateOdoTrack(double currentDeg, double flywheelRPM) {
        activeLockSource = LockSource.ODO;

        if (follower == null || follower.getPose() == null) return;
        Pose pose = follower.getPose();

        double targetX, targetY;
        if (aimTarget == AimTarget.OBELISK) {
            targetX = obeliskX;
            targetY = obeliskY;
        } else {
            int goalTag = getAllianceGoalTagId();
            targetX = (goalTag == RED_GOAL_TAG_ID) ? redGoalX : blueGoalX;
            targetY = (goalTag == RED_GOAL_TAG_ID) ? redGoalY : blueGoalY;
        }

        double desiredDeg = computeFieldPointTurretDeg(
                pose.getX(), pose.getY(), pose.getHeading(), targetX, targetY);

        // Smooth stepping for odo lock
        double errorDeg = computeAimErrorDeg(desiredDeg, currentDeg);
        if (Math.abs(errorDeg) <= odoDeadbandDeg) return;

        double stepDeg = clamp(errorDeg * odoKp, -odoMaxStepDeg, odoMaxStepDeg);
        stepDeg *= computeEdgeCorrectionScale(currentDeg);

        double targetDeg = normalizeDegrees(currentDeg + stepDeg);
        double distanceIn = computeDistanceToPoint(pose, targetX, targetY);
        targetDeg = applyShotLead(targetDeg, distanceIn, flywheelRPM, currentDeg);
        targetDeg = clampToSafeRange(normalizeDegrees(targetDeg));

        controller.setTargetAngleImmediate(targetDeg);
    }

    //---------------- AUTO Mode (Vision + Odo Fallback) ----------------

    private void updateAuto(Vision vision, double currentDeg, double flywheelRPM) {
        if (aimTarget == AimTarget.OBELISK) {
            hasFilteredTx = false;
            activeLockSource = LockSource.ODO;
            updateOdoTrack(currentDeg, flywheelRPM);
            return;
        }

        int requiredTagId = getRequiredGoalTagId(vision);

        if (vision != null && vision.seesTag(requiredTagId)) {
            // Vision available
            if (!wasVisionActive) {
                // Was in fallback; apply reacquire delay
                if (inFallback && visionReacquireTimer.milliseconds() < visionReacquireDelayMs) {
                    // Still in reacquire delay — keep odo
                    activeLockSource = LockSource.ODO;
                    updateOdoTrack(currentDeg, flywheelRPM);
                    return;
                }
                // Reacquire delay passed or was not in fallback
                inFallback = false;
                wasVisionActive = true;
                hasFilteredTx = false;
            }

            activeLockSource = LockSource.TX;
            double rawTx = vision.getTxForTag(requiredTagId);
            double filteredTx = filterTx(rawTx);
            double distanceIn = vision.getDistanceInchesForTag(requiredTagId);

            double targetDeg = computeVisionTargetDeg(filteredTx, distanceIn, currentDeg);
            targetDeg = applyShotLead(targetDeg, distanceIn, flywheelRPM, currentDeg);
            targetDeg = clampToSafeRange(normalizeDegrees(targetDeg));

            controller.setTargetAngleImmediate(targetDeg);
            computeChassisYawAssist(filteredTx, distanceIn, currentDeg);

            visionLostTimer.reset(); // reset fallback timer while we have vision
        } else {
            // Vision lost
            if (wasVisionActive) {
                wasVisionActive = false;
                visionReacquireTimer.reset();
            }

            if (visionLostTimer.milliseconds() < visionFallbackDelayMs) {
                // Short dropout — hold current position
                return;
            }

            // Full fallback to odo
            hasFilteredTx = false;
            inFallback = true;
            activeLockSource = LockSource.ODO;
            updateOdoTrack(currentDeg, flywheelRPM);
        }
    }

    //---------------- Vision Math ----------------

    private double computeVisionTargetDeg(double txDeg, double distanceIn, double currentDeg) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) {
            return currentDeg; // no correction needed
        }

        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        correctionDeg = clamp(correctionDeg, -visionMaxStepDeg, visionMaxStepDeg);

        if (turretWrapEnabled && needsOppositeWrapDirection(correctionDeg, currentDeg)) {
            correctionDeg = -Math.signum(correctionDeg)
                    * Math.max(Math.abs(correctionDeg), visionMinStepDeg);
        }

        correctionDeg *= computeEdgeCorrectionScale(currentDeg);

        if (Math.abs(correctionDeg) < visionMinStepDeg) {
            return currentDeg;
        }

        return normalizeDegrees(currentDeg + correctionDeg);
    }

    private double computeVisionCorrectionDeg(double txDeg, double distanceIn) {
        return computeVisionAimErrorDeg(txDeg, distanceIn) * visionKp * visionDirection;
    }

    private double computeVisionAimErrorDeg(double txDeg, double distanceIn) {
        double parallaxDeg = computeParallaxCorrectionDeg(distanceIn) * Math.signum(txDeg);
        boolean isFar = distanceIn > visionDistanceSplitIn;
        double biasDeg;
        if (GlobalVariables.isBlueAlliance()) {
            biasDeg = isFar ? blueVisionFarBiasDeg : blueVisionCloseBiasDeg;
        } else {
            biasDeg = isFar ? redVisionFarBiasDeg : redVisionCloseBiasDeg;
        }
        return txDeg + parallaxDeg + biasDeg;
    }

    private double computeParallaxCorrectionDeg(double distanceIn) {
        double safeDistance = Math.max(1.0, distanceIn);
        return Math.toDegrees(Math.atan2(cameraLateralOffsetIn, safeDistance));
    }

    private double filterTx(double txDeg) {
        double alpha = clamp(visionTxAlpha, 0.0, 1.0);
        if (!hasFilteredTx) {
            filteredTxDeg = txDeg;
            hasFilteredTx = true;
            return filteredTxDeg;
        }
        filteredTxDeg = (alpha * txDeg) + ((1.0 - alpha) * filteredTxDeg);
        return filteredTxDeg;
    }

    //---------------- Chassis Yaw Assist ----------------

    private void computeChassisYawAssist(double txDeg, double distanceIn, double currentDeg) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) {
            chassisYawCorrection = 0.0;
            return;
        }
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        boolean atMin = currentDeg <= (Math.min(turretMinDeg, turretMaxDeg) + limitAssistMarginDeg);
        boolean atMax = currentDeg >= (Math.max(turretMinDeg, turretMaxDeg) - limitAssistMarginDeg);
        boolean needsAssist = (atMin && correctionDeg < 0.0) || (atMax && correctionDeg > 0.0);

        if (needsAssist) {
            chassisYawCorrection = clamp(txDeg * chassisYawKp, -chassisYawMax, chassisYawMax);
        } else {
            chassisYawCorrection = 0.0;
        }
    }

    /**
     * True when the turret is at a travel limit and vision is asking to move further out.
     */
    public boolean needsChassisYawAssist(double txDeg, double distanceIn, double currentDeg) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) return false;
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        boolean atMin = currentDeg <= (Math.min(turretMinDeg, turretMaxDeg) + limitAssistMarginDeg);
        boolean atMax = currentDeg >= (Math.max(turretMinDeg, turretMaxDeg) - limitAssistMarginDeg);
        return (atMin && correctionDeg < 0.0) || (atMax && correctionDeg > 0.0);
    }

    //---------------- Shot Lead ----------------

    private double applyShotLead(double targetDeg, double distanceIn, double flywheelRPM,
                                  double currentDeg) {
        if (!shotLeadEnabled || follower == null || follower.getPose() == null) {
            return targetDeg;
        }

        // Get chassis velocity from the follower (Pedro provides velocity components)
        Pose pose = follower.getPose();
        // Pedro's Follower uses getVelocityVector() which returns a Vector for path velocity,
        // but for chassis velocity we need the raw velocity from the localizer.
        // The pose derivatives give dx/dt, dy/dt in field frame.
        // For now, we use a finite-difference approach within the aim controller's loop.
        // TODO: Wire up direct pinpoint velocity when available via PedroPathing API.
        // For initial implementation, shot lead requires external velocity injection.
        return targetDeg;
    }

    /**
     * Apply shot lead with externally provided chassis velocity.
     * Call this from Outtake.update() when chassis velocity is available.
     */
    public double applyShotLeadExternal(double targetDeg, double distanceIn, double flywheelRPM,
                                         double currentDeg, double chassisVxInPerSec,
                                         double chassisVyInPerSec, double robotHeadingRad) {
        if (!shotLeadEnabled) return targetDeg;

        double offset = shotLeadCalc.calculateLeadOffsetDeg(
                chassisVxInPerSec, chassisVyInPerSec,
                robotHeadingRad, turretForwardDeg, currentDeg,
                distanceIn, flywheelRPM);
        return targetDeg + offset;
    }

    //---------------- Odometry Math ----------------

    private double computeFieldPointTurretDeg(double robotX, double robotY, double robotHeadingRad,
                                              double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double headingToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        double relativeDeg = wrapSignedDegrees(headingToTargetDeg - robotHeadingDeg);
        return normalizeDegrees(turretForwardDeg + relativeDeg);
    }

    private double computeDistanceToPoint(Pose pose, double targetX, double targetY) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns the desired turret heading for aiming at the goal using odometry.
     */
    public double getOdoGoalDesiredHeadingDeg(Vision vision) {
        if (follower == null || follower.getPose() == null) return Double.NaN;
        Pose pose = follower.getPose();
        int goalTagId = getRequiredGoalTagId(vision);
        double targetX = (goalTagId == RED_GOAL_TAG_ID) ? redGoalX : blueGoalX;
        double targetY = (goalTagId == RED_GOAL_TAG_ID) ? redGoalY : blueGoalY;
        return computeFieldPointTurretDeg(pose.getX(), pose.getY(), pose.getHeading(), targetX, targetY);
    }

    /**
     * Returns the signed heading delta from current turret heading to the odo goal target.
     */
    public double getOdoGoalHeadingDeltaDeg(Vision vision, double currentDeg) {
        double desiredDeg = getOdoGoalDesiredHeadingDeg(vision);
        if (Double.isNaN(desiredDeg)) return Double.NaN;
        return computeAimErrorDeg(desiredDeg, currentDeg);
    }

    //---------------- Wrap / Launch Zone ----------------

    private void refreshWrapState() {
        if (follower == null || follower.getPose() == null) {
            inLaunchZone = false;
            turretWrapEnabled = false;
            return;
        }
        Pose pose = follower.getPose();
        inLaunchZone = isInsideLaunchZone(pose.getX(), pose.getY());
        if (!autoEnableWrapInLaunchZone) {
            turretWrapEnabled = false;
            return;
        }
        turretWrapEnabled = !wrapWithinLaunchZoneOnly || inLaunchZone;
    }

    private boolean isInsideLaunchZone(double x, double y) {
        return isInsideTriangle(x, y,
                goalLaunchBaseLeftX, goalLaunchBaseY,
                goalLaunchBaseRightX, goalLaunchBaseY,
                goalLaunchApexX, goalLaunchApexY)
                || isInsideTriangle(x, y,
                audienceLaunchBaseLeftX, audienceLaunchBaseY,
                audienceLaunchBaseRightX, audienceLaunchBaseY,
                audienceLaunchApexX, audienceLaunchApexY);
    }

    private boolean isInsideTriangle(double px, double py,
                                     double ax, double ay,
                                     double bx, double by,
                                     double cx, double cy) {
        double d1 = signedArea(px, py, ax, ay, bx, by);
        double d2 = signedArea(px, py, bx, by, cx, cy);
        double d3 = signedArea(px, py, cx, cy, ax, ay);
        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(hasNeg && hasPos);
    }

    private double signedArea(double px, double py,
                              double ax, double ay,
                              double bx, double by) {
        return (px - bx) * (ay - by) - (ax - bx) * (py - by);
    }

    //---------------- Aim Error with Wrap ----------------

    private double computeAimErrorDeg(double desiredDeg, double currentDeg) {
        double errorDeg = wrapSignedDegrees(desiredDeg - currentDeg);
        if (!turretWrapEnabled) return errorDeg;

        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        boolean desiredInRange = desiredDeg >= minDeg && desiredDeg <= maxDeg;
        boolean currentInRange = currentDeg >= minDeg && currentDeg <= maxDeg;
        if (!desiredInRange || !currentInRange) return errorDeg;

        boolean shortestCrossesWrapGap = (errorDeg < 0.0 && desiredDeg > currentDeg)
                || (errorDeg > 0.0 && desiredDeg < currentDeg);
        if (!shortestCrossesWrapGap) return errorDeg;

        return errorDeg > 0.0 ? (errorDeg - 360.0) : (errorDeg + 360.0);
    }

    private double computeEdgeCorrectionScale(double currentDeg) {
        double margin = Math.max(1.0, edgeCorrectionMarginDeg);
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        double distToNearLimit = Math.min(Math.abs(currentDeg - minDeg), Math.abs(maxDeg - currentDeg));
        double t = clamp(distToNearLimit / margin, 0.0, 1.0);
        return edgeCorrectionMinScale + (1.0 - edgeCorrectionMinScale) * t;
    }

    private boolean needsOppositeWrapDirection(double correctionDeg, double currentDeg) {
        boolean atMin = currentDeg <= (Math.min(turretMinDeg, turretMaxDeg) + limitAssistMarginDeg);
        boolean atMax = currentDeg >= (Math.max(turretMinDeg, turretMaxDeg) - limitAssistMarginDeg);
        return (atMin && correctionDeg < 0.0) || (atMax && correctionDeg > 0.0);
    }

    //---------------- Tag Helpers ----------------

    private int getRequiredGoalTagId(Vision vision) {
        if (vision != null) {
            int tagId = vision.getRequiredTagId();
            if (tagId == BLUE_GOAL_TAG_ID || tagId == RED_GOAL_TAG_ID) return tagId;
        }
        return getAllianceGoalTagId();
    }

    private int getAllianceGoalTagId() {
        return GlobalVariables.isBlueAlliance() ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
    }

    //---------------- Utility ----------------

    private double clampToSafeRange(double degrees) {
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        return clamp(degrees, minDeg, maxDeg);
    }

    private static double normalizeDegrees(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    private static double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
