package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Vision implements Subsystem {
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

    //---------------- Hardware ----------------
    private Limelight3A limelight;

    //---------------- Software ----------------
    public LLResult latest; //Cached result each loop
    public int currentPipeline = 0; //Current pipeline index (0..9)
    public double lastTx = 0;
    public double lastTy = 0;
    public double lastDistance = 100;
    public static double tagTxSign = 1.0;
    private int requiredTagId = -1; // -1 means "any tag"
    private int motifTagId = -1; // -1 means motif not selected

    //---------------- Constructor ----------------
    public Vision(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");
    }

    //---------------- Methods ----------------
    private void limelightInit(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
    }

    private void limelightUpdate(){
        latest = limelight.getLatestResult();
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
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            Pose3D mt2 = latest.getBotpose_MT2();
            if (mt2 != null) {
                return mt2;
            }
        } catch (Throwable ignored) {
        }
        try {
            return latest.getBotpose();
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
