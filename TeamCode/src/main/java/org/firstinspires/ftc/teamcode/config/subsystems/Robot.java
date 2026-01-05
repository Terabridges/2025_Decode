package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {

    //---------------- Objects ----------------
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public Gamepad gp1;
    public Gamepad gp2;
    public static VoltageSensor voltageSensor;

    //---------------- Subsystems ----------------
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;
    public Vision vision;

    public List<Subsystem> subsystems;

    // Relocalization gating defaults (tune as needed)
    public double relocalizeMaxAmbiguity = 0.2;
    public double relocalizeMaxHeadingErrorDeg = 15.0;
    public double relocalizeMaxPositionErrorIn = 18.0;
    public double relocalizeMaxSpeedInPerSec = 3.0;

    //---------------- Constructors ----------------
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2){
        this(hardwareMap, telemetry, gp1, gp2, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2, boolean autoMode){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        vision = new Vision(hardwareMap);
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = autoMode ? new ShooterAuto(hardwareMap, vision) : new Shooter(hardwareMap, vision);
        transfer = new Transfer(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(drive, intake, shooter, transfer, vision));

        this.gp1 = gp1;
        this.gp2 = gp2;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, null, null, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean autoMode){
        this(hardwareMap, telemetry, null, null, autoMode);
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    //---------------- Interface Methods ----------------
    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }

    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }

    public boolean relocalizeFollowerFromLimelight(Follower follower) {
        if (follower == null || vision == null) {
            return false;
        }
        if (!vision.hasTarget()) {
            return false;
        }
        Pose3D botPose = vision.getBotPoseMT2();
        if (botPose == null) {
            return false;
        }
        double ambiguity = vision.getBestPoseAmbiguity();
        if (ambiguity > relocalizeMaxAmbiguity) {
            return false;
        }
        double xIn = botPose.getPosition().x * 39.3701;
        double yIn = botPose.getPosition().y * 39.3701;
        double headingRad = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

        Pose currentPose = follower.getPose();
        double positionError = Math.hypot(xIn - currentPose.getX(), yIn - currentPose.getY());
        if (positionError > relocalizeMaxPositionErrorIn) {
            return false;
        }
        double headingErrorRad = smallestAngleDiff(headingRad, currentPose.getHeading());
        if (Math.toDegrees(Math.abs(headingErrorRad)) > relocalizeMaxHeadingErrorDeg) {
            return false;
        }
        double speed = follower.getVelocity().getMagnitude();
        if (speed > relocalizeMaxSpeedInPerSec) {
            return false;
        }

        follower.setPose(new Pose(xIn, yIn, headingRad));
        return true;
    }

    private static double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2.0 * Math.PI;
        while (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff;
    }
}
