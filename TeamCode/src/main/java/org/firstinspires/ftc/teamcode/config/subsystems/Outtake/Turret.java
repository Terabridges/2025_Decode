package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.Util;

@Configurable
public class Turret implements Subsystem {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    public enum AimMode {
        MANUAL,
        LOCK
    }

    public enum LockSource {
        NONE,
        TX,
        ODO
    }

    //---------------- Hardware ----------------
    private Servo leftTurret;
    private Servo rightTurret;
    private AnalogInput turretAnalog;
    private AbsoluteAnalogEncoder turretEnc;
    private Util util;


    //---------------- Software ----------------
    private double turretPos = 0;
    private double turretDegree = turretPos*360;
    public static double turretMinDeg = 18.0;
    public static double turretMaxDeg = 330.0;
    public static double turretForwardDeg = 163.0;
    public static double turretVelocity = 0;
    public static double velocityLoopTime = 250;
    // Conservative defaults to reduce lock oscillation from frame-to-frame overcorrection.
    public static double visionKp = 0.27;
    public static double visionDeadbandDeg = 1.2;
    public static double visionMaxStepDeg = 1.2;
    public static double visionMinStepDeg = 0.16;
    // Exponential smoothing for tx; 1.0 = no filtering, lower = smoother.
    // Default is no filtering so teleop behavior matches VisionTurretLockTester.
    public static double visionTxAlpha = 0.3;
    public static double cameraLateralOffsetIn = 0.0;
    public static double visionDirection = 1.0; // set to -1.0 to invert lock direction
    public static double blueVisionCloseBiasDeg = 3;
    public static double blueVisionFarBiasDeg = 2.5;
    public static double redVisionCloseBiasDeg = 0;
    public static double redVisionFarBiasDeg = 1.5;
    public static double visionDistanceSplitIn = 110.0;
    public static double limitAssistMarginDeg = 1.0;
    public static double visionFallbackDelayMs = 180.0;
    public static double visionReacquireDelayMs = 140.0;
    public static double odoHoldAfterFallbackMs = 220.0;
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 144.0;
    public static double redGoalX = 144.0;
    public static double redGoalY = 144.0;
    public static double odoKp = 0.95;
    public static double odoDeadbandDeg = 1.0;
    public static double odoMaxStepDeg = 4.5;
    public static double edgeCorrectionMinScale = 0.65;
    public static double edgeCorrectionMarginDeg = 35.0;
    public static boolean autoEnableWrapInLaunchZone = true;
    public static boolean wrapWithinLaunchZoneOnly = true;
    // DECODE launch zones in 144x144 in field coordinates (official dimensions).
    // Goal-side zone: 6 tiles wide and 3 tiles deep triangle.
    public static double goalLaunchBaseLeftX = 0.0;
    public static double goalLaunchBaseRightX = 144.0;
    public static double goalLaunchBaseY = 144.0;
    public static double goalLaunchApexX = 72.0;
    public static double goalLaunchApexY = 72.0;
    // Audience-side zone: 2 tiles wide and 1 tile deep centered triangle.
    public static double audienceLaunchBaseLeftX = 48.0;
    public static double audienceLaunchBaseRightX = 96.0;
    public static double audienceLaunchBaseY = 0.0;
    public static double audienceLaunchApexX = 72.0;
    public static double audienceLaunchApexY = 24.0;
    public static boolean invertRightServo = false;
    public static double leftServoOffset = 0.0;
    public static double rightServoOffset = 0.0;
    private AimMode aimMode = AimMode.MANUAL;
    private LockSource activeLockSource = LockSource.NONE;
    private boolean hasFilteredTx = false;
    private double filteredTxDeg = 0.0;
    private double commandedTurretDeg = 180.0;
    private double currentPosition = 0;
    private ElapsedTime velocityTimer;
    private ElapsedTime missingTargetTimer;
    private ElapsedTime seenTargetTimer;
    private ElapsedTime odoHoldTimer;
    private boolean turretWrapEnabled = false;
    private boolean inLaunchZone = false;


    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(Servo.class, "turretL");
        rightTurret = map.get(Servo.class, "turretR");
        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        commandedTurretDeg = rightTurret.getPosition() * 360.0;
        velocityTimer = new ElapsedTime();
        missingTargetTimer = new ElapsedTime();
        seenTargetTimer = new ElapsedTime();
        odoHoldTimer = new ElapsedTime();
        util = new Util();
    }

    //---------------- Methods ----------------
    public void setTurretPos(double pos){
        double basePos = util.clamp(pos, 0.0, 1.0);
        commandedTurretDeg = normalizeDegrees(basePos * 360.0);
        double leftPos = util.clamp(basePos + leftServoOffset, 0.0, 1.0);
        double rightBase = invertRightServo ? (1.0 - basePos) : basePos;
        double rightPos = util.clamp(rightBase + rightServoOffset, 0.0, 1.0);
        leftTurret.setPosition(leftPos);
        rightTurret.setPosition(rightPos);
    }

    public void setTurretDegree(double degree){
        refreshWrapStateFromOdometry();
        setTurretPos(clampToSafeRange(normalizeDegrees(degree)) / 360.0);
    }

    public double normalizeDegrees(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    /**
     * Restrict turret commands to safe wiring range [turretMinDeg, turretMaxDeg].
     */
    public double clampToSafeRange(double degrees) {
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        return util.clamp(degrees, minDeg, maxDeg);
    }

    public boolean isTurretWrapEnabled() {
        return turretWrapEnabled;
    }

    public boolean isInLaunchZone() {
        return inLaunchZone;
    }

    public boolean atMinLimit(double marginDeg) {
        return getCurrentDegrees() <= (Math.min(turretMinDeg, turretMaxDeg) + marginDeg);
    }

    public boolean atMaxLimit(double marginDeg) {
        return getCurrentDegrees() >= (Math.max(turretMinDeg, turretMaxDeg) - marginDeg);
    }

    public double computeParallaxCorrectionDeg(double distanceIn) {
        double safeDistance = Math.max(1.0, distanceIn);
        return Math.toDegrees(Math.atan2(cameraLateralOffsetIn, safeDistance));
    }

    public double computeVisionCorrectionDeg(double txDeg, double distanceIn) {
        return computeVisionAimErrorDeg(txDeg, distanceIn) * visionKp * visionDirection;
    }

    public double computeVisionAimErrorDeg(double txDeg, double distanceIn) {
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

    public boolean isVisionOnTarget(Vision vision, double toleranceDeg) {
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

    /**
     * True when vision asks to move further outward while already at a safe turret limit.
     */
    public boolean needsChassisYawAssist(double txDeg, double distanceIn) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) {
            return false;
        }
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        return (atMinLimit(limitAssistMarginDeg) && correctionDeg < 0.0)
                || (atMaxLimit(limitAssistMarginDeg) && correctionDeg > 0.0);
    }

    /**
     * Aim turret at a field point using robot field pose.
     */
    public void aimAtFieldPoint(double robotX, double robotY, double robotHeadingRad,
                                double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double headingToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        // Convert field bearing into signed robot-relative angle, then map into measured turret frame.
        double relativeDeg = ((headingToTargetDeg - robotHeadingDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        double turretDeg = normalizeDegrees(turretForwardDeg + relativeDeg);
        setTurretDegree(turretDeg);
    }

    /**
     * Vision lock controller for positional-servo turret.
     * Applies an incremental correction from tx (+ optional parallax correction from distance).
     */
    public void aimFromVision(double txDeg, double distanceIn) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) {
            return;
        }
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        correctionDeg = util.clamp(correctionDeg, -visionMaxStepDeg, visionMaxStepDeg);
        if (turretWrapEnabled && needsOppositeWrapDirection(correctionDeg)) {
            correctionDeg = -Math.signum(correctionDeg) * Math.max(Math.abs(correctionDeg), visionMinStepDeg);
        }
        correctionDeg *= computeEdgeCorrectionScale();
        if (Math.abs(correctionDeg) < visionMinStepDeg) {
            return;
        }
        double targetDeg = normalizeDegrees(getCurrentDegrees() + correctionDeg);
        setTurretDegree(targetDeg);
    }

    public void toggleTxLock() {
        setTxLockEnabled(!isTxLockEnabled());
    }

    public void setTxLockEnabled(boolean enabled) {
        if (enabled) {
            aimMode = AimMode.LOCK;
            // Avoid manual velocity nudge fighting lock.
            turretVelocity = 0.0;
            hasFilteredTx = false;
            activeLockSource = LockSource.NONE;
            missingTargetTimer.reset();
            seenTargetTimer.reset();
            odoHoldTimer.reset();
        } else if (aimMode == AimMode.LOCK) {
            aimMode = AimMode.MANUAL;
            activeLockSource = LockSource.NONE;
        }
    }

    public boolean isTxLockEnabled() {
        return aimMode == AimMode.LOCK;
    }

    public void toggleOdoLock() {
        setOdoLockEnabled(!isOdoLockEnabled());
    }

    public void setOdoLockEnabled(boolean enabled) {
        // Compatibility alias: unified lock now handles tx+odo fallback.
        setTxLockEnabled(enabled);
    }

    public boolean isOdoLockEnabled() {
        return isTxLockEnabled() && activeLockSource == LockSource.ODO;
    }

    public AimMode getAimMode() {
        return aimMode;
    }

    public LockSource getActiveLockSource() {
        return activeLockSource;
    }

    public void toggleAimLock() {
        toggleTxLock();
    }

    public void setAimLockEnabled(boolean enabled) {
        setTxLockEnabled(enabled);
    }

    public boolean isAimLockEnabled() {
        return isTxLockEnabled();
    }

    public void updateAimLock(Vision vision) {
        if (!isAimLockEnabled()) return;
        boolean hasVision = hasRequiredVisionTarget(vision);
        if (hasVision) {
            // Prevent immediate source flip back to TX during fast turns / blur.
            if (activeLockSource == LockSource.ODO && odoHoldTimer.milliseconds() < odoHoldAfterFallbackMs) {
                updateOdoLock(vision);
                return;
            }
            if (seenTargetTimer.milliseconds() >= visionReacquireDelayMs && tryVisionLock(vision)) {
                missingTargetTimer.reset();
                return;
            }
            // Target is present but not yet stable enough for TX; hold current source.
            if (activeLockSource == LockSource.ODO) {
                updateOdoLock(vision);
            }
            return;
        }
        seenTargetTimer.reset();
        // Avoid aggressive fallback when tag briefly drops out during fast turns.
        if (missingTargetTimer.milliseconds() < visionFallbackDelayMs) {
            return;
        }
        hasFilteredTx = false;
        if (activeLockSource != LockSource.ODO) {
            activeLockSource = LockSource.ODO;
            odoHoldTimer.reset();
        }
        updateOdoLock(vision);
    }

    private boolean hasRequiredVisionTarget(Vision vision) {
        if (vision == null) return false;
        int requiredTagId = vision.getRequiredTagId();
        return requiredTagId >= 0 ? vision.seesTag(requiredTagId) : vision.hasTarget();
    }

    private boolean tryVisionLock(Vision vision) {
        if (vision == null) return false;
        int requiredTagId = vision.getRequiredTagId();
        boolean hasTarget = requiredTagId >= 0
                ? vision.seesTag(requiredTagId)
                : vision.hasTarget();
        if (!hasTarget) return false;

        double tx = requiredTagId >= 0
                ? vision.getTxForTag(requiredTagId)
                : vision.getTx();
        tx = filterTx(tx);
        double distanceIn = requiredTagId >= 0
                ? vision.getDistanceInchesForTag(requiredTagId)
                : vision.getDistanceInches();
        activeLockSource = LockSource.TX;
        aimFromVision(tx, distanceIn);
        return true;
    }

    public void updateTxLock(Vision vision) {
        // Compatibility alias: unified lock now handles tx+odo fallback.
        updateAimLock(vision);
    }

    public void updateOdoLock() {
        updateOdoLock(null);
    }

    private void updateOdoLock(Vision vision) {
        if (follower == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;
        refreshWrapStateFromOdometry(robotPose);

        int requiredTagId = (vision != null) ? vision.getRequiredTagId() : -1;
        boolean aimBlueGoal;
        if (requiredTagId == BLUE_GOAL_TAG_ID) {
            aimBlueGoal = true;
        } else if (requiredTagId == RED_GOAL_TAG_ID) {
            aimBlueGoal = false;
        } else {
            aimBlueGoal = GlobalVariables.isBlueAlliance();
        }

        if (aimBlueGoal) {
            aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), blueGoalX, blueGoalY);
        } else {
            aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), redGoalX, redGoalY);
        }
    }

    private void aimAtFieldPointSmoothed(double robotX, double robotY, double robotHeadingRad,
                                         double targetX, double targetY) {
        double desiredDeg = computeFieldPointTurretDeg(robotX, robotY, robotHeadingRad, targetX, targetY);
        double errorDeg = computeAimErrorDeg(desiredDeg, getCurrentDegrees());
        if (Math.abs(errorDeg) <= odoDeadbandDeg) {
            return;
        }
        double stepDeg = util.clamp(errorDeg * odoKp, -odoMaxStepDeg, odoMaxStepDeg);
        stepDeg *= computeEdgeCorrectionScale();
        setTurretDegree(getCurrentDegrees() + stepDeg);
    }

    private double computeFieldPointTurretDeg(double robotX, double robotY, double robotHeadingRad,
                                              double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double headingToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        double relativeDeg = wrapSignedDegrees(headingToTargetDeg - robotHeadingDeg);
        return normalizeDegrees(turretForwardDeg + relativeDeg);
    }

    private double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private double filterTx(double txDeg) {
        double alpha = util.clamp(visionTxAlpha, 0.0, 1.0);
        if (!hasFilteredTx) {
            filteredTxDeg = txDeg;
            hasFilteredTx = true;
            return filteredTxDeg;
        }
        filteredTxDeg = (alpha * txDeg) + ((1.0 - alpha) * filteredTxDeg);
        return filteredTxDeg;
    }

    public void setTurretWithVelocity(){
        if (turretVelocity != 0 && velocityTimer.milliseconds() >= velocityLoopTime) {
            // Velocity nudges are also bounded by the same soft limits.
            setTurretDegree(getCurrentDegrees() + turretVelocity);
            velocityTimer.reset();
        }
    }

    public double getCurrentDegrees(){
        return commandedTurretDeg;
    }

    private double computeEdgeCorrectionScale() {
        double margin = Math.max(1.0, edgeCorrectionMarginDeg);
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        double currentDeg = getCurrentDegrees();
        double distToNearLimit = Math.min(Math.abs(currentDeg - minDeg), Math.abs(maxDeg - currentDeg));
        double t = util.clamp(distToNearLimit / margin, 0.0, 1.0);
        return edgeCorrectionMinScale + (1.0 - edgeCorrectionMinScale) * t;
    }

    private boolean needsOppositeWrapDirection(double correctionDeg) {
        return (atMinLimit(limitAssistMarginDeg) && correctionDeg < 0.0)
                || (atMaxLimit(limitAssistMarginDeg) && correctionDeg > 0.0);
    }

    private double computeAimErrorDeg(double desiredDeg, double currentDeg) {
        double errorDeg = wrapSignedDegrees(desiredDeg - currentDeg);
        if (!turretWrapEnabled) {
            return errorDeg;
        }
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        boolean desiredInRange = desiredDeg >= minDeg && desiredDeg <= maxDeg;
        boolean currentInRange = currentDeg >= minDeg && currentDeg <= maxDeg;
        if (!desiredInRange || !currentInRange) {
            return errorDeg;
        }
        // If shortest-path crosses through the forbidden wrap gap, choose the long-way path.
        boolean shortestCrossesWrapGap = (errorDeg < 0.0 && desiredDeg > currentDeg)
                || (errorDeg > 0.0 && desiredDeg < currentDeg);
        if (!shortestCrossesWrapGap) {
            return errorDeg;
        }
        return errorDeg > 0.0 ? (errorDeg - 360.0) : (errorDeg + 360.0);
    }

    public double getEncoderDegrees() {
        return turretEnc.getCurrentPosition();
    }

    public double getEncoderVoltage() {
        return turretAnalog.getVoltage();
    }

    public double getVelocityTimerMs(){
        return velocityTimer.milliseconds();
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        velocityTimer.reset();
    }

    @Override
    public void update(){
        refreshWrapStateFromOdometry();
        if (aimMode == AimMode.MANUAL) {
            setTurretWithVelocity();
        }
    }

    private void refreshWrapStateFromOdometry() {
        if (follower == null) {
            inLaunchZone = false;
            turretWrapEnabled = false;
            return;
        }
        Pose robotPose = follower.getPose();
        refreshWrapStateFromOdometry(robotPose);
    }

    private void refreshWrapStateFromOdometry(Pose robotPose) {
        if (robotPose == null) {
            inLaunchZone = false;
            turretWrapEnabled = false;
            return;
        }
        inLaunchZone = isInsideLaunchZone(robotPose.getX(), robotPose.getY());
        if (!autoEnableWrapInLaunchZone) {
            turretWrapEnabled = false;
            return;
        }
        turretWrapEnabled = wrapWithinLaunchZoneOnly ? inLaunchZone : true;
    }

    private boolean isInsideLaunchZone(double x, double y) {
        return isInsideTriangle(
                x, y,
                goalLaunchBaseLeftX, goalLaunchBaseY,
                goalLaunchBaseRightX, goalLaunchBaseY,
                goalLaunchApexX, goalLaunchApexY
        ) || isInsideTriangle(
                x, y,
                audienceLaunchBaseLeftX, audienceLaunchBaseY,
                audienceLaunchBaseRightX, audienceLaunchBaseY,
                audienceLaunchApexX, audienceLaunchApexY
        );
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

}
