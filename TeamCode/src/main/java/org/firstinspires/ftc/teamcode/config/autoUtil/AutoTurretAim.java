package org.firstinspires.ftc.teamcode.config.autoUtil;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

public class AutoTurretAim {
    private final Robot robot;
    private final AutoPoses poses;
    private final Alliance alliance;
    private final Range range;
    private final Telemetry telemetry;

    public AutoTurretAim(Robot robot, AutoPoses poses, Alliance alliance, Range range, Telemetry telemetry) {
        this.robot = robot;
        this.poses = poses;
        this.alliance = alliance;
        this.range = range;
        this.telemetry = telemetry;
    }

    public void updateAim(AutoStates activeState, boolean preloadComplete) {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null || robot.outtake.vision == null) return;

        if (activeState == AutoStates.ACQUIRE_MOTIF) {
            robot.outtake.turret.turretVelocity = 0;
            aimAtObelisk();
            telemetry.addData("Obelisk Aim", true);
        } else if (robot.outtake.vision.hasRequiredTarget()) {
            robot.outtake.turret.turretVelocity = 0;
            int requiredTag = robot.outtake.vision.getRequiredTagId();
            double tx = robot.outtake.vision.getTxForTag(requiredTag);
            double distance = robot.outtake.vision.getDistanceInchesForTag(requiredTag);
            robot.outtake.turret.aimFromVision(tx, distance);
            telemetry.addData("Lock Aim", true);
            telemetry.addData("Lock Tx", tx);
            telemetry.addData("Lock Distance In", distance);
        } else {
            robot.outtake.turret.turretVelocity = 0;
            aimAtGoal(preloadComplete);
            telemetry.addData("Goal Aim", true);
        }
        telemetry.addData("Required Tag Id", robot.outtake.vision.getRequiredTagId());
    }

    public void aimAtObelisk() {
        aimTurretAt(poses.getObeliskPose(alliance));
    }

    public void aimAtGoal(boolean preloadComplete) {
        aimTurretAt(poses.getGoalPose(alliance, range, preloadComplete));
    }

    private void aimTurretAt(Pose target) {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null || follower == null || target == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;
        robot.outtake.turret.aimAtFieldPoint(
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getHeading(),
                target.getX(),
                target.getY()
        );
    }
}
