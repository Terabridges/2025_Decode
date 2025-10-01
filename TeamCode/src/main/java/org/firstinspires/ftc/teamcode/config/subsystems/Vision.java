package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision implements Subsystem{

    //---------------- Hardware ----------------
    public NormalizedColorSensor ballInspector;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    //---------------- Software ----------------
    public LLResult latest; //Cached result each loop
    public int currentPipeline = 0; //Current pipeline index (0..9)
    private double robotYawDeg = 0.0;        // Chassis yaw from pinpoint

    //---------------- Constructor ----------------
    public Vision(HardwareMap map) {
        ballInspector = map.get(NormalizedColorSensor.class, "ball_inspector");
        limelight = map.get(Limelight3A.class, "limelight");
        pinpoint =  map.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    //---------------- Methods ----------------
    private void limelightInit(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
    }

    private void limelightUpdate(){
        if (pinpoint != null) {
            robotYawDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        }

        limelight.updateRobotOrientation(robotYawDeg);

        latest = limelight.getLatestResult();
    }

    public void pipeline(int index) {
        currentPipeline = Range.clip(index, 0, 9);
        if (limelight != null) limelight.pipelineSwitch(currentPipeline);
    }

    public boolean hasTarget() { return latest != null && latest.isValid(); }

    public double getTx() {
        return latest.getTx();
    }

    public double getTy() {
        return latest.getTy();
    }

    public Pose3D getBotPoseMT2() {
        if (hasTarget()) {
            try { return latest.getBotpose_MT2(); } catch (Throwable ignored) {}
        }
        return null;
    }

    public Pose3D getBotPoseMT1() {
        if (hasTarget()) {
            try { return latest.getBotpose(); } catch (Throwable ignored) {}
        }
        return null;
    }

    public void pinpointInit(){
        pinpoint.setOffsets(0, 0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void pinpointUpdate(){
        pinpoint.update();
    }

    public double getDistanceInches()
    {
        LLResultTypes.FiducialResult f0   = latest.getFiducialResults().get(0);
        Pose3D p = f0.getTargetPoseCameraSpace();   // meters
        double z = p.getPosition().z;               // forward distance (meters)
        return z * 39.3701;
    }

    public double getPlanarDistanceInches()
    {
        LLResultTypes.FiducialResult f0   = latest.getFiducialResults().get(0);
        Pose3D p = f0.getTargetPoseCameraSpace();   // AprilTag pose in the CAMERA frame (meters)
        double x = p.getPosition().x;               // +X = right (meters)
        double z = p.getPosition().z;               // +Z = forward (meters)
        return Math.hypot(x, z) * 39.3701;
    }

    public double getCameraBearingDeg()
    {
        LLResultTypes.FiducialResult f0   = latest.getFiducialResults().get(0);
        Pose3D p = f0.getTargetPoseCameraSpace();   // meters
        double x = p.getPosition().x;
        double z = p.getPosition().z;
        return Math.toDegrees(Math.atan2(x, z));
    }

    public double getDistanceUsingTan(){
        double targetOffsetAngle_Vertical = latest.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        limelightInit();
        pinpointInit();
    }

    @Override
    public void update(){
        limelightUpdate();
        pinpointUpdate();
    }
}
