package org.firstinspires.ftc.teamcode.config.archive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class VisionSubSWithMT2 implements Subsystem {

    //---------------- Hardware ----------------
    //public NormalizedColorSensor ballInspector;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    //---------------- Software ----------------
    public Shooter shooter; //This WONT work, must pass as a parameter
    public LLResult latest; //Cached result each loop
    public int currentPipeline = 0; //Current pipeline index (0..9)
    public double yawOffset = 0.0;

    //LLCO Limelight Chassis Offsets
    double LLCOx = 0;
    double LLCOy = 0;
    double LLCOz = 0;
    double LLCOyaw = 0;
    double LLCOpitch = 0;
    double LLCOroll = 0;

    //---------------- Constructor ----------------
    public VisionSubSWithMT2(HardwareMap map) {
        //ballInspector = map.get(NormalizedColorSensor.class, "ball_inspector");
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

        // ////////////////
        /*
        TODO:
        In order to use MegaTag2, think of the turret like a "mini robot"
        Give mount specifications in the limelight dashboard relative to turret rotation point instead of robot center

        Feed yaw as turret yaw relative to field by doing
        [imu (chassis) yaw relative to field] + [turret (servo encoder) yaw relative to chassis]
        OR SIMPLY: chassis yaw + turret yaw = theta (and then feed theta to updateRobotOrientation)

        Then, after you find bot pose of the mini robot or turret, you can find bot pose of the chassis via an offset
        The position of the chassis center to turret center is known and constant
        (Make sure to switch back to chassis yaw from imu for the final bot pose of chassis)

        */

        limelight.updateRobotOrientation(getJoinedYaw());

        latest = limelight.getLatestResult();
    }

    public void pipeline(int index) {
        currentPipeline = Range.clip(index, 0, 9);
        if (limelight != null) limelight.pipelineSwitch(currentPipeline);
    }

    public boolean hasTarget() { return latest != null && latest.isValid(); }

    /*
    Note: Limelight is mounted 90 degrees counterclockwise, therefore
    get tx returns ty
    get ty returns -tx
    */

    public double getTx() {
        if (hasTarget()) { return latest.getTy(); }
        return 0.0;
    }

    public double getTy() {
        if (hasTarget()) { return -latest.getTx(); }
        return 0.0;
    }

    public Pose3D getBotPoseMT2() {
        if (hasTarget()) { return latest.getBotpose_MT2(); }
        return null;
    }

    public Pose3D getBotPoseMT1() {
        if (hasTarget()) { return latest.getBotpose(); }
        return null;
    }

    private void pinpointInit(){
        pinpoint.setOffsets(5.70866, -1.527559, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    private void pinpointUpdate(){
        pinpoint.update();
    }

    public double getJoinedYaw()
    {
        double chassisYawDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        double turretYawDeg  = Math.toDegrees(shooter.getTurretHeading());
        return wrapDeg( (chassisYawDeg + turretYawDeg + yawOffset) ); // offset if θ=0 isn’t aligned
    }

    public Pose3D translateBotPose(Pose3D Pose)
    {
        Pose3D fieldToPivot = getBotPoseMT2();
        if (fieldToPivot == null) return null;

        // 1) Pivot yaw (deg) from Pose3D (Pose3D stores radians; convert)
        double pivotYawDeg = Math.toDegrees(fieldToPivot.getOrientation().getYaw());

        // 2) Rotate fixed pivot->chassis offset into FIELD by pivot yaw
        double c = Math.cos(Math.toRadians(pivotYawDeg));
        double s = Math.sin(Math.toRadians(pivotYawDeg));
        double dFx =  c * LLCOx - s * LLCOy;
        double dFy =  s * LLCOx + c * LLCOy;
        double dFz =  LLCOz;

        // 3) Translate position
        double xR = fieldToPivot.getPosition().x + dFx;
        double yR = fieldToPivot.getPosition().y + dFy;
        double zR = fieldToPivot.getPosition().z + dFz;
        Position pos = new Position(DistanceUnit.METER, xR, yR, zR, 0);

        // 4) Heading: trust Pinpoint (deg -> rad). Keep LL roll/pitch (rad).
        double chassisYawDeg = wrapDeg(pinpoint.getHeading(AngleUnit.DEGREES));
        double rollRad  = fieldToPivot.getOrientation().getRoll();
        double pitchRad = fieldToPivot.getOrientation().getPitch();

        YawPitchRollAngles ypr = new YawPitchRollAngles(
                AngleUnit.RADIANS,
                rollRad,
                pitchRad,
                Math.toRadians(chassisYawDeg),
                0
        );

        return new Pose3D(pos, ypr);
    }

    public double getDistanceInches()
    {
        if (hasTarget())
        {
            LLResultTypes.FiducialResult f0 = latest.getFiducialResults().get(0);
            Pose3D p = f0.getTargetPoseCameraSpace();   // meters
            double z = p.getPosition().z;               // forward distance (meters)
            return z * 39.3701;
        }
        return 0.0;
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

    private double wrapDeg(double a){
        while (a <= -180) a += 360;
        while (a >   180) a -= 360;
        return a;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        pinpointInit();
        limelightInit();
    }

    @Override
    public void update(){
        pinpointUpdate();
        limelightUpdate();
    }
}
