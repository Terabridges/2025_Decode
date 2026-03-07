package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.ShooterData;

public class Outtake implements Subsystem {
    public enum AimSource {
        NONE,
        ODO
    }

    public enum AimTarget {
        GOAL,
        OBELISK
    }

    //---------------- Hardware ----------------
    public Shooter shooter;
    public Turret turret;
    public Vision vision;
    private ShooterData shooterData;
    public double distanceInches = 0;

    //---------------- Software ----------------
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 144.0;
    public static double redGoalX = 144.0;
    public static double redGoalY = 144.0;
    public static double obeliskX = 72.0;
    public static double obeliskY = 144.0;

    private boolean aimLockEnabled = false;
    private AimSource activeAimSource = AimSource.NONE;
    private AimTarget aimTarget = AimTarget.GOAL;


    //---------------- Constructor ----------------
    public Outtake(HardwareMap map) {
        shooter = new Shooter(map);
        turret = new Turret(map);
        vision = new Vision(map);
        shooterData = new ShooterData();
    }

    //---------------- Methods ----------------
    public void toggleAimLock() {
        setAimLockEnabled(!isAimLockEnabled());
    }

    public void setAimLockEnabled(boolean enabled) {
        aimLockEnabled = enabled;
        if (!enabled) {
            activeAimSource = AimSource.NONE;
        }
    }

    public boolean isAimLockEnabled() {
        return aimLockEnabled;
    }

    public AimSource getActiveLockSource() {
        return activeAimSource;
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

    public void updateAimLock() {
        if (!aimLockEnabled) {
            activeAimSource = AimSource.NONE;
            return;
        }

        if (aimTarget == AimTarget.OBELISK) {
            aimAtObeliskWithOdometry();
        } else {
            aimAtGoalWithOdometry();
        }
    }

    public void aimAtObeliskWithOdometry() {
        Pose pose = (follower != null) ? follower.getPose() : null;
        if (pose == null) {
            activeAimSource = AimSource.NONE;
            return;
        }
        activeAimSource = AimSource.ODO;
        aimAtFieldPoint(pose, obeliskX, obeliskY);
    }

    public void aimAtGoalWithOdometry() {
        Pose pose = (follower != null) ? follower.getPose() : null;
        if (pose == null) {
            activeAimSource = AimSource.NONE;
            return;
        }
        activeAimSource = AimSource.ODO;

        if (GlobalVariables.isBlueAlliance()) {
            aimAtFieldPoint(pose, blueGoalX, blueGoalY);
        } else {
            aimAtFieldPoint(pose, redGoalX, redGoalY);
        }
    }

    public boolean isVisionOnTarget(Vision vision, double toleranceDeg) {
        // Vision aiming logic is intentionally not implemented in Outtake yet.
        return false;
    }

    private void aimAtFieldPoint(Pose robotPose, double targetX, double targetY) {
        double desiredDeg = computeFieldPointTurretDeg(robotPose, targetX, targetY);
        turret.setTurretDegree(desiredDeg);
    }

    private double computeFieldPointTurretDeg(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        double headingToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        double relativeDeg = wrapSignedDegrees(headingToTargetDeg - robotHeadingDeg);
        return turret.normalizeDegrees(Turret.turretForwardDeg + relativeDeg);
    }

    private double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        shooter.toInit();
        turret.toInit();
        vision.toInit();
    }

    @Override
    public void update(){
        distanceInches = vision.getDistanceInches();
        shooter.flywheelTargetRPM = shooterData.getRPMVal(distanceInches);
        shooter.hoodPos = shooterData.getAngleVal(distanceInches);
        shooter.update();
        turret.update();
        vision.update();
        updateAimLock();
    }

}
