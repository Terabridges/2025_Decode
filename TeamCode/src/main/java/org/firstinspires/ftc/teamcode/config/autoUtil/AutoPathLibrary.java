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
    private static final double SCORE_SMOOTH_END_DISTANCE_IN = 6.0;
    private static final double CLOSE_LOOP_PICKUP_STAGE_X_OFFSET_IN = 5.0;

    public AutoPathLibrary(AutoPoses poses) {
        this.poses = poses;
    }

    public PathChain goToPickup(Pose currentPose, Alliance alliance, Range range, int absoluteRow) {
        return buildLinear(currentPose, poses.getPickupStart(alliance, range, absoluteRow));
    }

    public PathChain row2GoToPickup(Pose currentPose, Alliance alliance, Range range) {
        Pose endPose = poses.getPickupStart(alliance, range, 2);
        return buildCurve(currentPose, poses.getRow2GoToScoreControl(alliance), endPose);
    }

    public PathChain pickup(Pose currentPose, Alliance alliance, Range range, int absoluteRow) {
        if (absoluteRow == 4) {
            Pose step1 = poses.getPickupRow4Step1(alliance);
            Pose step2 = poses.getPickupEnd(alliance, range, absoluteRow);
            return buildLinearTwoStep(currentPose, step1, step2);
        }
        return buildLinear(currentPose, poses.getPickupEnd(alliance, range, absoluteRow));
    }

    public PathChain farPickupZone(Pose currentPose, Alliance alliance) {
        return buildLinear(currentPose, poses.getFarPickupZone(alliance));
    }

    public PathChain closeLoopPickup(Pose currentPose, Alliance alliance) {
        Pose pickupPose = poses.getCloseLoopPickup(alliance);
        double stageX = (alliance == Alliance.BLUE)
                ? pickupPose.getX() + CLOSE_LOOP_PICKUP_STAGE_X_OFFSET_IN
                : pickupPose.getX() - CLOSE_LOOP_PICKUP_STAGE_X_OFFSET_IN;
        Pose stagePose = new Pose(stageX, pickupPose.getY(), pickupPose.getHeading());

        return buildCurve(
                currentPose,
                poses.getCloseLoopPickupControl(alliance),
                stagePose
        );
    }

    public PathChain closeLoopPickupPart2(Pose currentPose, Alliance alliance) {
        return buildLinear(currentPose, poses.getCloseLoopPickup(alliance));
    }

    public PathChain closeLoopCompletePickup(Pose currentPose, Alliance alliance) {
        return buildCurve(
                currentPose,
                poses.getCloseLoopCompletePickupControl(alliance),
                poses.getCloseLoopCompletePickup(alliance)
        );
    }

    public PathChain goToScore(Pose currentPose, Pose scorePose) {
        return buildLinearSmoothEnd(currentPose, scorePose, SCORE_SMOOTH_END_DISTANCE_IN);
    }

    public PathChain closeLoopGoToShoot(Pose currentPose, Alliance alliance, Pose shootPose, boolean useFinalShootControl) {
        // All close-loop go-to-shoot paths use the same control point.
        return buildCurve(currentPose, poses.getCloseLoopGoToScoreControl(alliance), shootPose);
    }

    public PathChain row2GoToShoot(Pose currentPose, Alliance alliance, Pose shootPose) {
        return buildCurve(currentPose, poses.getRow2GoToScoreControl(alliance), shootPose);
    }

    public PathChain releaseGoTo(Pose currentPose, Alliance alliance, Range range) {
        return buildLinear(currentPose, poses.getReleaseGoTo(alliance, range));
    }

    public PathChain releaseComplete(Pose currentPose, Alliance alliance, Range range) {
        return buildCurve(
                currentPose,
                poses.getReleaseGoTo(alliance, range),
                poses.getReleaseComplete(alliance, range)
        );
    }

    public PathChain leave(Pose currentPose, Alliance alliance, Range range) {
        Pose leavePose = poses.getLeave(alliance, range);
        if (AutoPoses.ReturnToStart) {
            return buildTurnThenDrive(currentPose, leavePose);
        }
        return buildLinear(currentPose, leavePose);
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

    public PathChain buildLinear(Pose start, Pose end) {
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

    private PathChain buildLinearSmoothEnd(Pose start, Pose end, double smoothDistanceIn) {
        if (follower == null || start == null || end == null) {
            return null;
        }

        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double dist = Math.hypot(dx, dy);

        if (!Double.isFinite(dist) || dist <= smoothDistanceIn + 0.5) {
            return buildLinear(start, end);
        }

        double ux = dx / dist;
        double uy = dy / dist;
        Pose preEnd = new Pose(
                end.getX() - ux * smoothDistanceIn,
                end.getY() - uy * smoothDistanceIn,
                end.getHeading()
        );

        return follower.pathBuilder()
                .addPath(new BezierLine(start, preEnd))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .addPath(new BezierLine(preEnd, end))
                .setLinearHeadingInterpolation(end.getHeading(), end.getHeading())
                .build();
    }
}
