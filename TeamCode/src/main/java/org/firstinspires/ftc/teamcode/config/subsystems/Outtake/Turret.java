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

    public enum AimTarget {
        GOAL,
        OBELISK
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
    public static double blueVisionCloseBiasDeg = 0;
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
    public static double obeliskX = 72.0;
    public static double obeliskY = 144.0;
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
    // Encoder->turret mapping calibration.
    public static double encoderRefTurretDeg = 59.25;
    public static double encoderRefDeg = 280.0;
    public static double encoderToTurretScale = 1.0; // turret-deg per encoder-deg near reference
    public static boolean encoderDirectionInverted = false;
    private AimMode aimMode = AimMode.MANUAL;
    private LockSource activeLockSource = LockSource.NONE;
    private AimTarget aimTarget = AimTarget.GOAL;
    private boolean hasFilteredTx = false;
    private double filteredTxDeg = 0.0;
    private double commandedTurretDeg = 180.0;
    private double currentPosition = 0;
    private ElapsedTime velocityTimer;
    private boolean turretWrapEnabled = false;
    private boolean inLaunchZone = false;


    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(Servo.class, "turretL");
        rightTurret = map.get(Servo.class, "turretR");
        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        util = new Util();
        syncCommandToMeasured();
        velocityTimer = new ElapsedTime();
    }

    //---------------- Methods ----------------
    public void setTurretPos(double pos){
        double basePos = util.clamp(pos, 0.0, 1.0);
        commandedTurretDeg = normalizeDegrees(basePos * 360.0);
        double leftPos = util.clamp(basePos + leftServoOffset, 0.0, 1.0);
        leftTurret.setPosition(leftPos);
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

    public AimTarget getAimTarget() {
        return aimTarget;
    }

    public void setAimTargetGoal() {
        aimTarget = AimTarget.GOAL;
    }

    public void setAimTargetObelisk() {
        aimTarget = AimTarget.OBELISK;
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
        if (!isAimLockEnabled()) {
            return;
        }
        if (aimTarget == AimTarget.OBELISK) {
            hasFilteredTx = false;
            activeLockSource = LockSource.ODO;
            aimAtObeliskWithOdometry();
            return;
        }

        int requiredGoalTagId = getRequiredGoalTagId(vision);
        if (vision != null && vision.seesTag(requiredGoalTagId)) {
            double tx = filterTx(vision.getTxForTag(requiredGoalTagId));
            double distanceIn = vision.getDistanceInchesForTag(requiredGoalTagId);
            activeLockSource = LockSource.TX;
            aimFromVision(tx, distanceIn);
            return;
        }

        hasFilteredTx = false;
        activeLockSource = LockSource.ODO;
        updateGoalOdoLock(requiredGoalTagId);
    }

    public void updateTxLock(Vision vision) {
        // Compatibility alias: unified lock now handles tx+odo fallback.
        updateAimLock(vision);
    }

    public void updateOdoLock() {
        updateGoalOdoLock(getAllianceGoalTagId());
    }

    public void aimAtObeliskWithOdometry() {
        if (follower == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;
        refreshWrapStateFromOdometry(robotPose);
        activeLockSource = LockSource.ODO;
        aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), obeliskX, obeliskY);
    }

    private void updateGoalOdoLock(int goalTagId) {
        if (follower == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;
        refreshWrapStateFromOdometry(robotPose);

        if (goalTagId == BLUE_GOAL_TAG_ID) {
            aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), blueGoalX, blueGoalY);
            return;
        }
        if (goalTagId == RED_GOAL_TAG_ID) {
            aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), redGoalX, redGoalY);
            return;
        }

        if (GlobalVariables.isBlueAlliance()) {
            aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), blueGoalX, blueGoalY);
        } else {
            aimAtFieldPointSmoothed(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), redGoalX, redGoalY);
        }
    }

    private int getRequiredGoalTagId(Vision vision) {
        if (vision != null) {
            int tagId = vision.getRequiredTagId();
            if (tagId == BLUE_GOAL_TAG_ID || tagId == RED_GOAL_TAG_ID) {
                return tagId;
            }
        }
        return getAllianceGoalTagId();
    }

    private int getAllianceGoalTagId() {
        return GlobalVariables.isBlueAlliance() ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
    }

    /**
     * Returns the absolute turret heading (deg) that ODO lock is trying to reach for goal aiming.
     * Returns NaN when follower pose is unavailable.
     */
    public double getOdoGoalDesiredHeadingDeg(Vision vision) {
        if (follower == null || follower.getPose() == null) {
            return Double.NaN;
        }
        Pose robotPose = follower.getPose();
        int goalTagId = getRequiredGoalTagId(vision);
        double targetX = (goalTagId == RED_GOAL_TAG_ID) ? redGoalX : blueGoalX;
        double targetY = (goalTagId == RED_GOAL_TAG_ID) ? redGoalY : blueGoalY;
        return computeFieldPointTurretDeg(
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getHeading(),
                targetX,
                targetY
        );
    }

    /**
     * Signed heading delta (deg) from current turret heading to ODO goal target heading.
     * Positive means rotate toward increasing turret angle; negative toward decreasing.
     * Returns NaN when follower pose is unavailable.
     */
    public double getOdoGoalHeadingDeltaDeg(Vision vision) {
        double desiredDeg = getOdoGoalDesiredHeadingDeg(vision);
        if (Double.isNaN(desiredDeg)) {
            return Double.NaN;
        }
        return computeAimErrorDeg(desiredDeg, getCurrentDegrees());
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

    public void syncCommandToMeasured() {
        double mappedEncoderDeg = getMappedEncoderTurretDegrees();
        if (!Double.isNaN(mappedEncoderDeg) && !Double.isInfinite(mappedEncoderDeg)) {
            commandedTurretDeg = normalizeDegrees(mappedEncoderDeg);
            return;
        }

        // Fallback to existing left-servo command frame when encoder is unavailable.
        double leftPos = util.clamp(leftTurret.getPosition(), 0.0, 1.0);
        double basePos = util.clamp(leftPos - leftServoOffset, 0.0, 1.0);
        commandedTurretDeg = normalizeDegrees(basePos * 360.0);
    }

    public double getMappedEncoderTurretDegrees() {
        double encoderDeg = getEncoderDegrees();
        if (Double.isNaN(encoderDeg) || Double.isInfinite(encoderDeg)) {
            return Double.NaN;
        }
        double deltaEncDeg = wrapSignedDegrees(encoderDeg - encoderRefDeg);
        if (encoderDirectionInverted) {
            deltaEncDeg = -deltaEncDeg;
        }
        double scale = (Math.abs(encoderToTurretScale) < 1e-6) ? 1.0 : encoderToTurretScale;
        return normalizeDegrees(encoderRefTurretDeg + (deltaEncDeg * scale));
    }

    public double getMappedEncoderErrorDeg(double targetTurretDeg) {
        double mapped = getMappedEncoderTurretDegrees();
        if (Double.isNaN(mapped)) {
            return Double.NaN;
        }
        return wrapSignedDegrees(mapped - normalizeDegrees(targetTurretDeg));
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
        syncCommandToMeasured();
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
