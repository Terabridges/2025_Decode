package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@PsiKitFieldAutoLog
public class Vision implements Subsystem {
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;
    public static boolean sendRobotYawToLimelight = true;
    public static boolean includeTurretRelativeYawInFeed = true;
    public static boolean useEncoderTurretYawForFeed = true;
    public static double turretRelativeYawSign = -1.0;
    public static double turretRelativeYawOffsetDeg = 0.0;
    public static double ftcRotatedFrameBaseDeg = 90.0;
    public static double robotYawSign = 1.0;
    public static double robotYawOffsetDeg = 0.0;
    public static double turretPivotRobotXMeter = -0.06985;
    public static double turretPivotRobotYMeter = 0.0;
    public static double cameraPoseRobotXMeter = -0.06985;
    public static double cameraPoseRobotYMeter = -0.070866;
    public static double cameraPoseRobotZMeter = 0.385191;
    public static double cameraPoseRobotYawBaseDeg = 0.0;
    public static double cameraPoseRobotPitchDeg = 12.5;
    public static double cameraPoseRobotRollDeg = 90.0;
    public static double cameraPoseRobotYawTurretSign = 1.0;
    public static double cameraPoseRobotYawOffsetDeg = 0.0;
    public static boolean rotateCameraPositionWithTurretYaw = true;
    public static double cameraPoseTurretOffsetXMeter = 0.0;
    public static double cameraPoseTurretOffsetZMeter = 0.0;
    public static double cameraPoseTurretOffsetYMeter = -0.070866;
    public static boolean applyTurretGeometryPoseCompensation = true;

    //---------------- Hardware ----------------
    private Limelight3A limelight;
    private Turret turret;

    //---------------- Software ----------------
    public LLResult latest; //Cached result each loop
    public int currentPipeline = 0; //Current pipeline index (0..9)
    public double lastTx = 0;
    public double lastTy = 0;
    public double lastDistance = 100;
    private double lastRobotYawSentDeg = Double.NaN;
    private double lastChassisYawDeg = Double.NaN;
    private double lastTurretRelativeYawDeg = 0.0;
    private double lastTurretRelativeYawFromCommandDeg = Double.NaN;
    private double lastTurretRelativeYawFromEncoderDeg = Double.NaN;
    private boolean lastRobotYawSendSuccess = false;
    public static double tagTxSign = 1.0;
    private int requiredTagId = -1; // -1 means "any tag"
    private int motifTagId = -1; // -1 means motif not selected
    private double lastCompBaselineCameraXMeter = Double.NaN;
    private double lastCompBaselineCameraYMeter = Double.NaN;
    private double lastCompCurrentCameraXMeter = Double.NaN;
    private double lastCompCurrentCameraYMeter = Double.NaN;
    private double lastCompBaselineYawDeg = Double.NaN;
    private double lastCompCurrentYawDeg = Double.NaN;
    private double lastCompDeltaYawDeg = Double.NaN;
    private double lastCompDeltaXRobotMeter = Double.NaN;
    private double lastCompDeltaYRobotMeter = Double.NaN;

    //---------------- Constructor ----------------
    public Vision(HardwareMap map) {
        this(map, null);
    }

    public Vision(HardwareMap map, Turret turret) {
        limelight = map.get(Limelight3A.class, "limelight");
        this.turret = turret;
    }

    //---------------- Methods ----------------
    private void limelightInit(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
    }

    private void limelightUpdate(){
        sendRobotYawIfAvailable();
        latest = limelight.getLatestResult();
    }

    private void sendRobotYawIfAvailable() {
        lastRobotYawSendSuccess = false;
        if (limelight == null) {
            logYawFeedToLogger();
            return;
        }

        double turretRelativeYawDeg = 0.0;
        lastTurretRelativeYawFromCommandDeg = Double.NaN;
        lastTurretRelativeYawFromEncoderDeg = Double.NaN;
        if (includeTurretRelativeYawInFeed && turret != null) {
            double fromCommand = wrapSignedDegrees(turret.getCurrentDegrees() - Turret.turretForwardDeg);
            lastTurretRelativeYawFromCommandDeg =
                    (fromCommand * turretRelativeYawSign) + turretRelativeYawOffsetDeg;

            double mappedTurretDeg = turret.getMappedEncoderTurretDegrees();
            if (!Double.isNaN(mappedTurretDeg) && !Double.isInfinite(mappedTurretDeg)) {
                double fromEncoder = wrapSignedDegrees(mappedTurretDeg - Turret.turretForwardDeg);
                lastTurretRelativeYawFromEncoderDeg =
                        (fromEncoder * turretRelativeYawSign) + turretRelativeYawOffsetDeg;
            }

            if (useEncoderTurretYawForFeed && !Double.isNaN(lastTurretRelativeYawFromEncoderDeg)) {
                turretRelativeYawDeg = lastTurretRelativeYawFromEncoderDeg;
            } else {
                turretRelativeYawDeg = lastTurretRelativeYawFromCommandDeg;
            }
        }
        lastTurretRelativeYawDeg = turretRelativeYawDeg;

        if (!sendRobotYawToLimelight || follower == null) {
            if (follower == null) {
                lastChassisYawDeg = Double.NaN;
                lastRobotYawSentDeg = Double.NaN;
            }
            logYawFeedToLogger();
            return;
        }

        double chassisYawDeg = Math.toDegrees(follower.getHeading());
        lastChassisYawDeg = chassisYawDeg;

        double yawDeg = chassisYawDeg + turretRelativeYawDeg;
        yawDeg += ftcRotatedFrameBaseDeg;
        yawDeg = AngleUnit.normalizeDegrees((yawDeg * robotYawSign) + robotYawOffsetDeg);
        lastRobotYawSentDeg = yawDeg;

        try {
            lastRobotYawSendSuccess = limelight.updateRobotOrientation(yawDeg);
        } catch (Throwable ignored) {
            lastRobotYawSendSuccess = false;
        }

        logYawFeedToLogger();
    }

    public double[] getTurretCompensatedPose2dMetersFromPose3d(Pose3D llPose) {
        if (llPose == null) {
            return null;
        }

        double rawX = llPose.getPosition().x;
        double rawY = llPose.getPosition().y;
        double rawHeadingDeg = llPose.getOrientation().getYaw(AngleUnit.DEGREES);
        double turretYawDeg = lastTurretRelativeYawDeg;

        if (!applyTurretGeometryPoseCompensation) {
            return new double[] { rawX, rawY, rawHeadingDeg };
        }

        double baselineYawDeg = AngleUnit.normalizeDegrees(cameraPoseRobotYawBaseDeg + cameraPoseRobotYawOffsetDeg);
        double currentYawDeg = AngleUnit.normalizeDegrees(
                cameraPoseRobotYawBaseDeg
                        + (turretYawDeg * cameraPoseRobotYawTurretSign)
                        + cameraPoseRobotYawOffsetDeg
        );

        double[] baselineTranslation = getCameraRobotTranslationMeters(0.0);
        double baselineX = baselineTranslation[0];
        double baselineY = baselineTranslation[1];

        double[] currentTranslation = getCameraRobotTranslationMeters(turretYawDeg);
        double currentX = currentTranslation[0];
        double currentY = currentTranslation[1];

        double deltaYawDeg = AngleUnit.normalizeDegrees(baselineYawDeg - currentYawDeg);
        double deltaYawRad = Math.toRadians(deltaYawDeg);
        double cosDelta = Math.cos(deltaYawRad);
        double sinDelta = Math.sin(deltaYawRad);

        double rotatedCurrentX = (cosDelta * currentX) - (sinDelta * currentY);
        double rotatedCurrentY = (sinDelta * currentX) + (cosDelta * currentY);
        double deltaXRobot = baselineX - rotatedCurrentX;
        double deltaYRobot = baselineY - rotatedCurrentY;

        lastCompBaselineCameraXMeter = baselineX;
        lastCompBaselineCameraYMeter = baselineY;
        lastCompCurrentCameraXMeter = currentX;
        lastCompCurrentCameraYMeter = currentY;
        lastCompBaselineYawDeg = baselineYawDeg;
        lastCompCurrentYawDeg = currentYawDeg;
        lastCompDeltaYawDeg = deltaYawDeg;
        lastCompDeltaXRobotMeter = deltaXRobot;
        lastCompDeltaYRobotMeter = deltaYRobot;

        double rawHeadingRad = Math.toRadians(rawHeadingDeg);
        double cosRaw = Math.cos(rawHeadingRad);
        double sinRaw = Math.sin(rawHeadingRad);

        double correctedX = rawX + (cosRaw * deltaXRobot) - (sinRaw * deltaYRobot);
        double correctedY = rawY + (sinRaw * deltaXRobot) + (cosRaw * deltaYRobot);
        double correctedHeadingDeg = AngleUnit.normalizeDegrees(rawHeadingDeg + deltaYawDeg);

        return new double[] { correctedX, correctedY, correctedHeadingDeg };
    }

    private double[] getCameraRobotTranslationMeters(double turretYawDeg) {
        if (!rotateCameraPositionWithTurretYaw) {
            return new double[] { cameraPoseRobotXMeter, cameraPoseRobotYMeter };
        }

        double turretRad = Math.toRadians(turretYawDeg);
        double cos = Math.cos(turretRad);
        double sin = Math.sin(turretRad);
        double rotatedOffsetX = (cameraPoseTurretOffsetXMeter * cos) - (cameraPoseTurretOffsetYMeter * sin);
        double rotatedOffsetY = (cameraPoseTurretOffsetXMeter * sin) + (cameraPoseTurretOffsetYMeter * cos);
        return new double[] {
                turretPivotRobotXMeter + rotatedOffsetX,
                turretPivotRobotYMeter + rotatedOffsetY
        };
    }

    private void logYawFeedToLogger() {
        Logger.recordOutput("Vision/LimelightYawFeed/ChassisYawDeg", lastChassisYawDeg);
        Logger.recordOutput("Vision/LimelightYawFeed/UseEncoderTurretYawForFeed", useEncoderTurretYawForFeed ? 1.0 : 0.0);
        Logger.recordOutput("Vision/LimelightYawFeed/TurretRelativeYawDeg", lastTurretRelativeYawDeg);
        Logger.recordOutput("Vision/LimelightYawFeed/TurretRelativeYawFromCommandDeg", lastTurretRelativeYawFromCommandDeg);
        Logger.recordOutput("Vision/LimelightYawFeed/TurretRelativeYawFromEncoderDeg", lastTurretRelativeYawFromEncoderDeg);
        if (!Double.isNaN(lastTurretRelativeYawFromCommandDeg) && !Double.isInfinite(lastTurretRelativeYawFromCommandDeg)) {
            Logger.recordOutput("Vision/LimelightYawFeed/Diag/ChassisPlusTurretCmdDeg",
                AngleUnit.normalizeDegrees(lastChassisYawDeg + lastTurretRelativeYawFromCommandDeg));
            Logger.recordOutput("Vision/LimelightYawFeed/Diag/ChassisMinusTurretCmdDeg",
                AngleUnit.normalizeDegrees(lastChassisYawDeg - lastTurretRelativeYawFromCommandDeg));
        }
        if (!Double.isNaN(lastTurretRelativeYawFromEncoderDeg) && !Double.isInfinite(lastTurretRelativeYawFromEncoderDeg)) {
            Logger.recordOutput("Vision/LimelightYawFeed/Diag/ChassisPlusTurretEncDeg",
                AngleUnit.normalizeDegrees(lastChassisYawDeg + lastTurretRelativeYawFromEncoderDeg));
            Logger.recordOutput("Vision/LimelightYawFeed/Diag/ChassisMinusTurretEncDeg",
                AngleUnit.normalizeDegrees(lastChassisYawDeg - lastTurretRelativeYawFromEncoderDeg));
        }
        Logger.recordOutput("Vision/LimelightYawFeed/FtcBaseDeg", ftcRotatedFrameBaseDeg);
        Logger.recordOutput("Vision/LimelightYawFeed/ExtraOffsetDeg", robotYawOffsetDeg);
        Logger.recordOutput("Vision/LimelightYawFeed/YawSentDeg", lastRobotYawSentDeg);
        Logger.recordOutput("Vision/LimelightYawFeed/SendSuccess", lastRobotYawSendSuccess ? 1.0 : 0.0);
        Logger.recordOutput("Vision/LimelightPoseComp/ApplyTurretGeometryPoseCompensation", applyTurretGeometryPoseCompensation ? 1.0 : 0.0);
        Logger.recordOutput("Vision/LimelightPoseComp/RotateCameraPositionWithTurretYaw", rotateCameraPositionWithTurretYaw ? 1.0 : 0.0);
        Logger.recordOutput("Vision/LimelightPoseComp/TurretPivotRobotXMeter", turretPivotRobotXMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/TurretPivotRobotYMeter", turretPivotRobotYMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/CameraTurretOffsetXMeter", cameraPoseTurretOffsetXMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/CameraTurretOffsetYMeter", cameraPoseTurretOffsetYMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/CameraRobotZMeter", cameraPoseRobotZMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/CameraRobotPitchDeg", cameraPoseRobotPitchDeg);
        Logger.recordOutput("Vision/LimelightPoseComp/CameraRobotRollDeg", cameraPoseRobotRollDeg);
        Logger.recordOutput("Vision/LimelightPoseComp/BaselineCameraXMeter", lastCompBaselineCameraXMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/BaselineCameraYMeter", lastCompBaselineCameraYMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/CurrentCameraXMeter", lastCompCurrentCameraXMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/CurrentCameraYMeter", lastCompCurrentCameraYMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/BaselineYawDeg", lastCompBaselineYawDeg);
        Logger.recordOutput("Vision/LimelightPoseComp/CurrentYawDeg", lastCompCurrentYawDeg);
        Logger.recordOutput("Vision/LimelightPoseComp/DeltaYawDeg", lastCompDeltaYawDeg);
        Logger.recordOutput("Vision/LimelightPoseComp/DeltaXRobotMeter", lastCompDeltaXRobotMeter);
        Logger.recordOutput("Vision/LimelightPoseComp/DeltaYRobotMeter", lastCompDeltaYRobotMeter);
    }

    public double getLastRobotYawSentDeg() {
        return lastRobotYawSentDeg;
    }

    public double getLastChassisYawDeg() {
        return lastChassisYawDeg;
    }

    public double getLastTurretRelativeYawDeg() {
        return lastTurretRelativeYawDeg;
    }

    public boolean wasLastRobotYawSendSuccessful() {
        return lastRobotYawSendSuccess;
    }

    private static double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    public void pipeline(int index) {
        currentPipeline = Range.clip(index, 0, 9);
        if (limelight != null) limelight.pipelineSwitch(currentPipeline);
    }

    public boolean hasTarget() { return latest != null && latest.isValid(); }

    public void setRequiredTagId(int tagId) {
        requiredTagId = tagId;
    }

    public int getRequiredTagId() {
        return requiredTagId;
    }

    public void setMotifTagId(int tagId) {
        motifTagId = tagId;
    }

    public int getMotifTagId() {
        return motifTagId;
    }

    public void clearMotifTagId() {
        motifTagId = -1;
    }

    /** True if any fiducial in the current frame matches tagId; tagId < 0 accepts any target. */
    public boolean seesTag(int tagId) {
        if (tagId < 0) return hasTarget();
        if (!hasTarget() || latest.getFiducialResults().isEmpty()) return false;
        for (LLResultTypes.FiducialResult f : latest.getFiducialResults()) {
            if (f.getFiducialId() == tagId) {
                return true;
            }
        }
        return false;
    }

    public boolean hasRequiredTarget() {
        return seesTag(requiredTagId);
    }

    private LLResultTypes.FiducialResult getFiducialById(int tagId) {
        if (!hasTarget() || latest.getFiducialResults().isEmpty()) {
            return null;
        }
        for (LLResultTypes.FiducialResult f : latest.getFiducialResults()) {
            if (f.getFiducialId() == tagId) {
                return f;
            }
        }
        return null;
    }

    /**
     * Returns camera bearing in degrees for the requested tag if visible.
     * Falls back to the frame-level tx cache when the specific tag isn't visible.
     */
    public double getTxForTag(int tagId) {
        if (tagId < 0) {
            return getTx();
        }
        LLResultTypes.FiducialResult f = getFiducialById(tagId);
        if (f != null) {
            // Use ROBOT-space pose so left/right sign is stable regardless of camera mount rotation.
            Pose3D p = f.getTargetPoseRobotSpace();
            double x = p.getPosition().x;
            double z = p.getPosition().z;
            lastTx = tagTxSign * Math.toDegrees(Math.atan2(x, z));
        }
        return lastTx;
    }

    /*
    Note: Limelight is mounted 90 degrees counterclockwise, therefore
    get tx returns ty
    get ty returns -tx
    */

    public double getTx() {
//        if (hasTarget()) { return latest.getTy(); }
//        return 0.0;
        if (hasTarget()) { lastTx = latest.getTy(); }
        return lastTx;
    }

    public double getTy() {
//        if (hasTarget()) { return -latest.getTx(); }
//        return 0.0;
        if (hasTarget()) { lastTy = -latest.getTx(); }
        return lastTy;
    }

    /** Returns the ID of the currently tracked fiducial, or -1 if none is visible. */
    public int getCurrentTagId() {
        if (!hasTarget() || latest.getFiducialResults().isEmpty()) {
            return -1;
        }
        return latest.getFiducialResults().get(0).getFiducialId();
    }

    /**
     * Returns visible goal tag ID (20 or 24), favoring the one closest to camera center.
     * Returns -1 when neither goal tag is currently visible.
     */
    public int getVisibleGoalTagId() {
        if (!hasTarget() || latest.getFiducialResults().isEmpty()) {
            return -1;
        }
        int bestId = -1;
        double bestScore = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult f : latest.getFiducialResults()) {
            int id = f.getFiducialId();
            if (id != BLUE_GOAL_TAG_ID && id != RED_GOAL_TAG_ID) continue;

            Pose3D p = f.getTargetPoseRobotSpace();
            double x = p.getPosition().x;
            double z = p.getPosition().z;
            double absBearingDeg = Math.abs(Math.toDegrees(Math.atan2(x, z)));
            if (absBearingDeg < bestScore) {
                bestScore = absBearingDeg;
                bestId = id;
            }
        }
        return bestId;
    }

    /**
     * Returns Limelight robot pose estimate (MT2 preferred, then standard botpose), else null.
     */
    public Pose3D getLatestBotPose() {
        Pose3D mt2 = getLatestMt2Pose();
        return (mt2 != null) ? mt2 : getLatestMt1Pose();
    }

    public Pose3D getLatestMt1Pose() {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose();
        } catch (Throwable ignored) {
            return null;
        }
    }

    public Pose3D getLatestMt2Pose() {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose_MT2();
        } catch (Throwable ignored) {
            return null;
        }
    }

    /** Choose the obelisk face (21/22/23) whose yaw is most directly facing the robot. */
    public int getFieldFacingObeliskId() {
        if (!hasTarget() || latest.getFiducialResults().isEmpty()) {
            return getCurrentTagId();
        }
        double bestScore = Double.MAX_VALUE;
        int bestId = -1;
        for (LLResultTypes.FiducialResult f : latest.getFiducialResults()) {
            int id = f.getFiducialId();
            if (id != 21 && id != 22 && id != 23) continue;
            Pose3D p = f.getTargetPoseRobotSpace();
            double yaw = Math.abs(p.getOrientation().getYaw(AngleUnit.DEGREES));
            if (yaw < bestScore) {
                bestScore = yaw;
                bestId = id;
            }
        }
        return (bestId != -1) ? bestId : getCurrentTagId();
    }

    public double getDistanceInches()
    {
//        if (hasTarget())
//        {
//            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
//            Pose3D p = f0.getTargetPoseCameraSpace();   // meters
//            double z = p.getPosition().z;               // forward distance (meters)
//            return z * 39.3701;
//        }
//        return 0.0;
        if (hasTarget())
        {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();   // meters
            double z = p.getPosition().z;               // forward distance (meters)
            z *= 39.3701;
            lastDistance = z;
        }
        return lastDistance;
    }

    public double getDistanceInchesForTag(int tagId) {
        if (tagId < 0) {
            return getDistanceInches();
        }
        LLResultTypes.FiducialResult f = getFiducialById(tagId);
        if (f != null) {
            Pose3D p = f.getTargetPoseCameraSpace();
            double z = p.getPosition().z;
            lastDistance = z * 39.3701;
        }
        return lastDistance;
    }

    public double getPlanarDistanceInches()
    {
        if (hasTarget()) {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();   // AprilTag pose in the CAMERA frame (meters)
            double x = p.getPosition().x;               // +X = right (meters)
            double z = p.getPosition().z;               // +Z = forward (meters)
            return Math.hypot(x, z) * 39.3701;
        }
        return 0.0;
    }

    public double getCameraBearingDeg()
    {
        if (hasTarget()) {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();   // meters
            double x = p.getPosition().x;
            double z = p.getPosition().z;
            return Math.toDegrees(Math.atan2(x, z));
        }
        return 0.0;
    }

    public double getFiducialX()
    {
        if (hasTarget()) {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();
            return p.getPosition().x;
        }
        return 0.0;
    }

    public double getFiducialY()
    {
        if (hasTarget()) {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();
            return p.getPosition().y;
        }
        return 0.0;
    }

    public double getFiducialZ()
    {
        if (hasTarget()) {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();
            return p.getPosition().z;
        }
        return 0.0;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        limelightInit();
    }

    @Override
    public void update(){
        limelightUpdate();
    }

}
