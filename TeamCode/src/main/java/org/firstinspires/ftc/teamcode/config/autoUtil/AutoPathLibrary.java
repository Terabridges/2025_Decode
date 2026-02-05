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

    public PathChain goToPickup(Pose currentPose, Alliance alliance, Range range, int rowIndex) {
        return buildLinear(currentPose, poses.getPickupStart(alliance, range, rowIndex), true);
    }

    public PathChain pickup(Pose currentPose, Alliance alliance, Range range, int rowIndex) {
        return buildLinear(currentPose, poses.getPickupEnd(alliance, range, rowIndex), false);
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
        return buildLinear(currentPose, poses.getLeave(alliance, range), false);
    }

    public PathChain buildLinear(Pose start, Pose end, boolean smoothEnd) {
        if (follower == null || start == null || end == null) {
            return null;
        }

        if (smoothEnd) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                    .setBrakingStart(0.75)
                    .setBrakingStrength(0.85)
                    .build();
        }

        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
