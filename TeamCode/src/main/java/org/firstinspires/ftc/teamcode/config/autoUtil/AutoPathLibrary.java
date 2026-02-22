package org.firstinspires.ftc.teamcode.config.autoUtil;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

public class AutoPathLibrary {
    private final AutoPoses poses;

    public AutoPathLibrary(AutoPoses poses) {
        this.poses = poses;
    }

    public PathChain goToPickup(Pose currentPose, Alliance alliance, int absoluteRow) {
        return buildLinear(currentPose, poses.getPickupStart(alliance, absoluteRow), true);
    }

    public PathChain pickup(Pose currentPose, Alliance alliance, int absoluteRow) {
        return buildLinear(currentPose, poses.getPickupEnd(alliance, absoluteRow), false);
    }

    public PathChain farPickupZone(Pose currentPose, Alliance alliance) {
        return buildLinear(currentPose, poses.getFarPickupZone(alliance), false);
    }

    public PathChain goToScore(Pose currentPose, Pose scorePose) {
        return buildLinear(currentPose, scorePose, true);
    }

    public PathChain releaseGoTo(Pose currentPose, Alliance alliance, Range range) {
        return buildLinear(currentPose, poses.getReleaseGoTo(alliance, range), false);
    }

    public PathChain releaseComplete(Pose currentPose, Alliance alliance, Range range) {
        return buildLinear(currentPose, poses.getReleaseComplete(alliance, range), false);
    }

    public PathChain leave(Pose currentPose, Alliance alliance, Range range) {
        Pose leavePose = poses.getLeave(alliance, range);
        if (AutoPoses.ReturnToStart) {
            return buildTurnThenDrive(currentPose, leavePose);
        }
        return buildLinear(currentPose, leavePose, false);
    }

    private PathChain buildTurnThenDrive(Pose start, Pose end) {
        if (follower == null || start == null || end == null) {
            return null;
        }

        // Tiny translation segment lets heading settle before the actual drive segment.
        double epsilon = 0.01;
        Pose turnPose = new Pose(start.getX() + epsilon, start.getY(), end.getHeading());

        return follower.pathBuilder()
                .addPath(new BezierLine(start, turnPose))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .addPath(new BezierLine(turnPose, end))
                .setLinearHeadingInterpolation(end.getHeading(), end.getHeading())
                .build();
    }

    public PathChain buildLinear(Pose start, Pose end, boolean smoothEnd) {
        if (follower == null || start == null || end == null) {
            return null;
        }

        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
