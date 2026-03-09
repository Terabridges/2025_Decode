package org.firstinspires.ftc.teamcode.config.autoUtil;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.BezierCurve;
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

    public PathChain goToPickup(Pose currentPose, Alliance alliance, Range range, int absoluteRow) {
        return buildLinear(currentPose, poses.getPickupStart(alliance, range, absoluteRow), true);
    }

    public PathChain pickup(Pose currentPose, Alliance alliance, Range range, int absoluteRow) {
        if (absoluteRow == 4) {
            Pose step1 = poses.getPickupRow4Step1(alliance);
            Pose step2 = poses.getPickupEnd(alliance, range, absoluteRow);
            return buildLinearTwoStep(currentPose, step1, step2);
        }
        return buildLinear(currentPose, poses.getPickupEnd(alliance, range, absoluteRow), false);
    }

    public PathChain farPickupZone(Pose currentPose, Alliance alliance) {
        return buildLinear(currentPose, poses.getFarPickupZone(alliance), false);
    }

    public PathChain closeLoopPickup(Pose currentPose, Alliance alliance) {
        return buildCurve(
                currentPose,
                poses.getCloseLoopSharedControl(alliance),
                poses.getCloseLoopPickup(alliance)
        );
    }

    public PathChain goToScore(Pose currentPose, Pose scorePose) {
        return buildLinear(currentPose, scorePose, true);
    }

    public PathChain closeLoopGoToShoot(Pose currentPose, Alliance alliance, Pose shootPose, boolean useFinalShootControl) {
        Pose control = useFinalShootControl
                ? poses.getCloseLoopFinalShootControl(alliance)
                : poses.getCloseLoopSharedControl(alliance);
        return buildCurve(currentPose, control, shootPose);
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

    private PathChain buildLinearTwoStep(Pose start, Pose mid, Pose end) {
        if (follower == null || start == null || mid == null || end == null) {
            return null;
        }

        return follower.pathBuilder()
                .addPath(new BezierLine(start, mid))
                .setLinearHeadingInterpolation(start.getHeading(), mid.getHeading())
                .addPath(new BezierLine(mid, end))
                .setLinearHeadingInterpolation(mid.getHeading(), end.getHeading())
                .build();
    }

    private PathChain buildCurve(Pose start, Pose control, Pose end) {
        if (follower == null || start == null || control == null || end == null) {
            return null;
        }

        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
