package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
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
    // Position-servo mode: 1.0 commands the full measured heading correction each loop.
    public static double visionKp = 1.0;
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
    public static double rightServoOffset = 0.021;
    /**
     * Reference base servo position used for degree<->servo conversion.
     * Base position is the shared, pre-offset command in [0..1] before left/right offsets.
     */
    public static double turretServoRefPos = 0.50;
    /** Turret heading (deg) that corresponds to {@link #turretServoRefPos}. */
    public static double turretServoRefTurretDeg = 180.0;
    /**
     * Calibrated slope (turret-deg per +1.0 base servo command).
     * Negative means increasing servo position decreases turret angle.
      * Example with -341.4: requesting +20 turret-deg changes base command by
      * deltaPos = +20 / -341.4 = -0.0586 (before right offset/inversion).
     */
    public static double turretDegPerServoCommand = -341.4;
    public static double turretServoPwmMinUs = 500.0;
    public static double turretServoPwmMaxUs = 2500.0;
    // Encoder->turret mapping calibration.
    public static double encoderRefTurretDeg = 59.25;
    public static double encoderRefDeg = 280.0;
    public static double encoderToTurretScale = 1.0; // turret-deg per encoder-deg near reference
    public static boolean encoderDirectionInverted = false;
    private AimMode aimMode = AimMode.MANUAL;
    private LockSource activeLockSource = LockSource.NONE;
    private AimTarget aimTarget = AimTarget.GOAL;
    private double commandedTurretDeg = 180.0;
    private double currentPosition = 0;
    private ElapsedTime velocityTimer;
    private boolean inLaunchZone = false;


    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(Servo.class, "turretL");
        rightTurret = map.get(Servo.class, "turretR");
        applyTurretServoPwmRange(leftTurret);
        applyTurretServoPwmRange(rightTurret);
        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        util = new Util();
        syncCommandToMeasured();
        velocityTimer = new ElapsedTime();
    }

    //---------------- Methods ----------------
    public void setTurretPos(double pos){
        refreshWrapStateFromOdometry();
        double requestedBasePos = clampBasePosToSharedRange(pos);
        double desiredDeg = baseServoPosToTurretDeg(requestedBasePos);
        double safeTargetDeg = resolveSafeTargetDeg(desiredDeg, getCurrentDegrees());
        double safeBasePos = turretDegToBaseServoPos(safeTargetDeg);
        applyTurretPosRaw(safeBasePos);
    }

    /** Applies configured PWM pulse range to a turret servo if supported by the device. */
    private void applyTurretServoPwmRange(Servo servo) {
        if (!(servo instanceof PwmControl)) return;
        double lo = Math.min(turretServoPwmMinUs, turretServoPwmMaxUs);
        double hi = Math.max(turretServoPwmMinUs, turretServoPwmMaxUs);
        ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(lo, hi));
    }

    public void setTurretDegree(double degree){
        setTurretPos(turretDegToBaseServoPos(normalizeDegrees(degree)));
    }

    private void applyTurretPosRaw(double basePos) {
        double clampedBasePos = util.clamp(basePos, 0.0, 1.0);
        commandedTurretDeg = baseServoPosToTurretDeg(clampedBasePos);

        double leftPos = clampedBasePos;
        double rightBasePos = invertRightServo ? (1.0 - clampedBasePos) : clampedBasePos;
        double rightPos = util.clamp(rightBasePos + rightServoOffset, 0.0, 1.0);

        leftTurret.setPosition(leftPos);
        rightTurret.setPosition(rightPos);
    }

    /**
     * Convert desired turret heading (deg) into shared base servo command in [0..1]
     * using reference-point + calibrated-slope mapping.
     */
    private double turretDegToBaseServoPos(double turretDeg) {
        double slope = (Math.abs(turretDegPerServoCommand) < 1e-6) ? 360.0 : turretDegPerServoCommand;
        double errorDeg = wrapSignedDegrees(normalizeDegrees(turretDeg) - normalizeDegrees(turretServoRefTurretDeg));
        return clampBasePosToSharedRange(turretServoRefPos + (errorDeg / slope));
    }

    private double getSharedBaseMin() {
        return Math.max(0.0, invertRightServo ? rightServoOffset : -rightServoOffset);
    }

    private double getSharedBaseMax() {
        return Math.min(1.0, invertRightServo ? 1.0 + rightServoOffset : 1.0 - rightServoOffset);
    }

    private double clampBasePosToSharedRange(double requestedBasePos) {
        return util.clamp(requestedBasePos, getSharedBaseMin(), getSharedBaseMax());
    }

    /**
     * Convert shared base servo command back into turret heading (deg) in the same
     * calibrated frame used by {@link #turretDegToBaseServoPos(double)}.
     */
    private double baseServoPosToTurretDeg(double basePos) {
        double slope = (Math.abs(turretDegPerServoCommand) < 1e-6) ? 360.0 : turretDegPerServoCommand;
        return normalizeDegrees(turretServoRefTurretDeg + ((basePos - turretServoRefPos) * slope));
    }

    public double normalizeDegrees(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    private boolean isWithinSafeRange(double degrees) {
        double normalizedDeg = normalizeDegrees(degrees);
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        return normalizedDeg >= minDeg && normalizedDeg <= maxDeg;
    }

    /**
     * Route target by choosing the best valid direction candidate (CW/CCW).
     * If both are invalid (target lies in deadzone), choose the closer safe edge.
     */
    private double resolveSafeTargetDeg(double desiredDeg, double currentDeg) {
        double desiredNorm = normalizeDegrees(desiredDeg);
        double currentNorm = normalizeDegrees(currentDeg);

        double cwSafeDist = clockwiseSafeDistance(currentNorm, desiredNorm);
        double ccwSafeDist = counterClockwiseSafeDistance(currentNorm, desiredNorm);
        if (Math.min(cwSafeDist, ccwSafeDist) < Double.POSITIVE_INFINITY) {
            return desiredNorm;
        }

        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        double distToMin = nearestSafeTravelDistance(currentNorm, minDeg);
        double distToMax = nearestSafeTravelDistance(currentNorm, maxDeg);
        return (distToMin <= distToMax) ? minDeg : maxDeg;
    }

    private double nearestSafeTravelDistance(double fromDeg, double toDeg) {
        double cw = clockwiseSafeDistance(fromDeg, toDeg);
        double ccw = counterClockwiseSafeDistance(fromDeg, toDeg);
        return Math.min(cw, ccw);
    }

    private double clockwiseSafeDistance(double fromDeg, double toDeg) {
        double from = normalizeDegrees(fromDeg);
        double to = normalizeDegrees(toDeg);
        if (!isWithinSafeRange(from) || !isWithinSafeRange(to)) return Double.POSITIVE_INFINITY;
        // In this turret's safe interval [min..max], clockwise safe travel must not wrap at 360->0.
        if (from > to) return Double.POSITIVE_INFINITY;
        return to - from;
    }

    private double counterClockwiseSafeDistance(double fromDeg, double toDeg) {
        double from = normalizeDegrees(fromDeg);
        double to = normalizeDegrees(toDeg);
        if (!isWithinSafeRange(from) || !isWithinSafeRange(to)) return Double.POSITIVE_INFINITY;
        // In this turret's safe interval [min..max], counterclockwise safe travel must not wrap at 0->360.
        if (from < to) return Double.POSITIVE_INFINITY;
        return from - to;
    }

    public boolean isTurretWrapEnabled() {
        return true;
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
     * Each loop, convert tx into an absolute turret heading and command that heading directly.
     */
    public void aimFromVision(double txDeg, double distanceIn) {
        double aimErrorDeg = computeVisionAimErrorDeg(txDeg, distanceIn);
        if (Math.abs(aimErrorDeg) <= visionDeadbandDeg) {
            return;
        }
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
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
            activeLockSource = LockSource.ODO;
            aimAtObeliskWithOdometry();
            return;
        }

        int requiredGoalTagId = getRequiredGoalTagId(vision);
        if (vision != null && vision.seesTag(requiredGoalTagId)) {
            double tx = vision.getTxForTag(requiredGoalTagId);
            double distanceIn = vision.getDistanceInchesForTag(requiredGoalTagId);
            activeLockSource = LockSource.TX;
            aimFromVision(tx, distanceIn);
            return;
        }

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
        aimAtFieldPoint(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), obeliskX, obeliskY);
    }

    private void updateGoalOdoLock(int goalTagId) {
        if (follower == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;
        refreshWrapStateFromOdometry(robotPose);

        if (goalTagId == BLUE_GOAL_TAG_ID) {
            aimAtFieldPoint(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), blueGoalX, blueGoalY);
            return;
        }
        if (goalTagId == RED_GOAL_TAG_ID) {
            aimAtFieldPoint(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), redGoalX, redGoalY);
            return;
        }

        if (GlobalVariables.isBlueAlliance()) {
            aimAtFieldPoint(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), blueGoalX, blueGoalY);
        } else {
            aimAtFieldPoint(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), redGoalX, redGoalY);
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

    private double computeAimErrorDeg(double desiredDeg, double currentDeg) {
        return wrapSignedDegrees(desiredDeg - currentDeg);
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
        double basePos = util.clamp(leftPos, 0.0, 1.0);
        commandedTurretDeg = baseServoPosToTurretDeg(basePos);
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
            return;
        }
        Pose robotPose = follower.getPose();
        refreshWrapStateFromOdometry(robotPose);
    }

    private void refreshWrapStateFromOdometry(Pose robotPose) {
        if (robotPose == null) {
            inLaunchZone = false;
            return;
        }
        inLaunchZone = isInsideLaunchZone(robotPose.getX(), robotPose.getY());
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
