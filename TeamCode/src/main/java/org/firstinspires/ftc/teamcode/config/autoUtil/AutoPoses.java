package org.firstinspires.ftc.teamcode.config.autoUtil;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

public class AutoPoses {
    public static boolean ReturnToStart = false;
    private static final double FIELD_SIZE = 144.0;
    private static final double ROBOT_WIDTH = 17.5;
    private static final double ROBOT_LENGTH = 18.0;

    private Pose poseDeg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private double mirrorHeadingDeg(double headingDeg) {
        return ((180.0 - headingDeg) % 360.0 + 360.0) % 360.0;
    }

    private Pose mirrorBluePose(Pose bluePose) {
        return poseDeg(
                FIELD_SIZE - bluePose.getX(),
                bluePose.getY(),
                mirrorHeadingDeg(Math.toDegrees(bluePose.getHeading()))
        );
    }

    // ===== Start Poses =====
    public Pose blueCloseStartPose = poseDeg(23.567, 124.916, 232.0);
    public Pose blueFarStartPose = poseDeg(48.0 + ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0, 0.0);
    public Pose redCloseStartPose = poseDeg(120.433, 124.916, 128);
    public Pose redFarStartPose = poseDeg(FIELD_SIZE - (48.0 + ROBOT_LENGTH / 2.0), ROBOT_WIDTH / 2.0, 0.0);

    // ===== Score Poses =====
    public Pose scoreCB = poseDeg(48.0, 96.0, 232.0);
    public Pose scoreLB = poseDeg(54.0, 16.0, 0.0);
    public Pose scoreCR = poseDeg(FIELD_SIZE - scoreCB.getX(), scoreCB.getY(), 128.0);
    public Pose scoreLR = poseDeg(FIELD_SIZE - scoreLB.getX(), scoreLB.getY(), 0.0);
    public Pose row2ShootCloseB = poseDeg(55.0, 96.0, 180.0);
    public Pose row2ShootCloseR = poseDeg(FIELD_SIZE - row2ShootCloseB.getX(), row2ShootCloseB.getY(), 180.0);
    public Pose finalShootCloseB = poseDeg(49.0, 120.0, 180.0);
    public Pose finalShootCloseR = poseDeg(FIELD_SIZE - finalShootCloseB.getX(), finalShootCloseB.getY(), 130.0);

    // ===== Load Poses =====
    public Pose loadB = poseDeg(ROBOT_WIDTH / 2.0, ROBOT_LENGTH / 2.0, 0);
    public Pose loadR = poseDeg(FIELD_SIZE - ROBOT_WIDTH / 2.0, ROBOT_LENGTH / 2.0, 0);

    // ===== Release Poses =====
    public Pose releaseGoToCloseB = poseDeg(25.0, 66.0, 180.0);
    public Pose releaseCompleteCloseB = poseDeg(17.5, 66.0, 180.0);
    public Pose releaseGoToCloseR = poseDeg(FIELD_SIZE - releaseGoToCloseB.getX(), releaseGoToCloseB.getY() + 5.0, 180.0);
    public Pose releaseCompleteCloseR = poseDeg(FIELD_SIZE - releaseCompleteCloseB.getX(), releaseCompleteCloseB.getY() + 5.0, 180.0);

    // ===== Leave Poses =====
    public Pose leaveCB = poseDeg(44.0, 115.0, 140.0);
    public Pose leaveLB = poseDeg(15.0, 15.0, 0.0);
    public Pose leaveCR = mirrorBluePose(leaveCB);
    public Pose leaveLR = poseDeg(FIELD_SIZE - leaveLB.getX(), leaveLB.getY(), 0.0);

    // ===== Pickup Start Poses =====
    public Pose pick1StartCB = poseDeg(44.0, 84.0, 180.0);
    public Pose pick1StartCR = poseDeg(FIELD_SIZE - pick1StartCB.getX(), pick1StartCB.getY(), 180.0);

    public Pose pick2StartLB = poseDeg(44.0, 60.0, 0.0);
    public Pose pick2StartCB = poseDeg(44.0, 60.0, 180.0);
    public Pose pick2StartCR = poseDeg((FIELD_SIZE - pick2StartCB.getX()) - 2.0, pick2StartCB.getY(), 180.0);
    public Pose pick2StartLR = poseDeg(FIELD_SIZE - pick2StartLB.getX(), pick2StartLB.getY(), 0.0);

    public Pose pick3StartLB = poseDeg(44.0, 36.0, 0.0);
    public Pose pick3StartCB = poseDeg(44.0, 36.0, 180.0);
    public Pose pick3StartCR = poseDeg(FIELD_SIZE - pick3StartCB.getX(), pick3StartCB.getY(), 180.0);
    public Pose pick3StartLR = poseDeg(FIELD_SIZE - pick3StartLB.getX(), pick3StartLB.getY(), 0.0);

    public Pose pick4StartLB = poseDeg(17.5, 16.0, 20.0);
    public Pose pick4StartLR = poseDeg(FIELD_SIZE - pick4StartLB.getX(), pick4StartLB.getY(), 340.0);
    public Pose pick4Step1LB = poseDeg(17.5, 12.5, 20.0);
    public Pose pick4Step1LR = poseDeg(FIELD_SIZE - pick4Step1LB.getX(), pick4Step1LB.getY(), 340.0);

    public Pose farPickupZoneB = poseDeg(15.5, 10.0, 0.0);
    public Pose farPickupZoneR = poseDeg(FIELD_SIZE - farPickupZoneB.getX(), farPickupZoneB.getY(), 0.0);
    public Pose farPickupZoneControlB = poseDeg(36.0, 7.0, 0.0);
    public Pose farPickupZoneControlR = poseDeg(FIELD_SIZE - farPickupZoneControlB.getX(), farPickupZoneControlB.getY(), 0.0);
    public Pose row4GoToPickupB = poseDeg(13.5, 16.0, 30.0);
    public Pose row4GoToPickupR = poseDeg(FIELD_SIZE - row4GoToPickupB.getX(), row4GoToPickupB.getY(), 330.0);
    public Pose row4IntermediatePickupB = poseDeg(13.5, 13.5, 30.0);
    public Pose row4IntermediatePickupR = poseDeg(FIELD_SIZE - row4IntermediatePickupB.getX(), row4IntermediatePickupB.getY(), 330.0);
    public Pose row4CompletePickupControlB = poseDeg(20.5, 11.5, 0.0);
    public Pose row4CompletePickupControlR = poseDeg(FIELD_SIZE - row4CompletePickupControlB.getX(), row4CompletePickupControlB.getY(), 0.0);
    public Pose row4CompletePickupB = poseDeg(13.0, 12.0, 12.0);
    public Pose row4CompletePickupR = poseDeg(FIELD_SIZE - row4CompletePickupB.getX(), row4CompletePickupB.getY(), 348.0);
    public Pose closeLoopPickupB = poseDeg(16.5, 62.0, 180.0);
    public Pose closeLoopPickupR = poseDeg(FIELD_SIZE - closeLoopPickupB.getX(), closeLoopPickupB.getY() + 2.0, 180.0);
    public Pose closeLoopPickupControlB = poseDeg(53.0, 60.0, 0.0);
    public Pose closeLoopPickupControlR = mirrorBluePose(closeLoopPickupControlB);
    public Pose closeLoopGoToScoreControlB = poseDeg(50.0, 55.0, 0.0);
    public Pose closeLoopGoToScoreControlR = mirrorBluePose(closeLoopGoToScoreControlB);
    public Pose row2GoToScoreControlB = poseDeg(53.0, 60.0, 0.0);
    public Pose row2GoToScoreControlR = mirrorBluePose(row2GoToScoreControlB);
    public Pose closeLoopCompletePickupB = poseDeg(15.0, 50.0, 145.0);
    public Pose closeLoopCompletePickupR = poseDeg(FIELD_SIZE - closeLoopCompletePickupB.getX(), closeLoopCompletePickupB.getY(), 215.0);
    public Pose closeLoopCompletePickupControlB = poseDeg(25.0, 55.0, 0.0);
    public Pose closeLoopCompletePickupControlR = mirrorBluePose(closeLoopCompletePickupControlB);
    public Pose closeLoopSharedControlB = poseDeg(33.0, 60.0, 0.0);
    public Pose closeLoopSharedControlR = mirrorBluePose(closeLoopSharedControlB);
    public Pose closeLoopFinalShootControlB = poseDeg(35.0, 81.0, 0.0);
    public Pose closeLoopFinalShootControlR = mirrorBluePose(closeLoopFinalShootControlB);

    // ===== Pickup End Poses =====
    public Pose pick1EndCB = poseDeg(24.0, 84.0, 180.0);
    public Pose pick1EndCR = poseDeg(FIELD_SIZE - pick1EndCB.getX(), pick1EndCB.getY(), 180.0);

    public Pose pick2EndLB = poseDeg(24.0, 60.0, 0.0);
    public Pose pick2EndCB = poseDeg(24.0, 60.0, 180.0);
    public Pose pick2EndCR = poseDeg(FIELD_SIZE - pick2EndCB.getX(), pick2EndCB.getY(), 180.0);
    public Pose pick2EndLR = poseDeg(FIELD_SIZE - pick2EndLB.getX(), pick2EndLB.getY(), 0.0);

    public Pose pick3EndLB = poseDeg(24.0, 36.0, 0.0);
    public Pose pick3EndCB = poseDeg(24.0, 36.0, 180.0);
    public Pose pick3EndCR = poseDeg(FIELD_SIZE - pick3EndCB.getX(), pick3EndCB.getY(), 180.0);
    public Pose pick3EndLR = poseDeg(FIELD_SIZE - pick3EndLB.getX(), pick3EndLB.getY(), 0.0);

    public Pose pick4EndLB = poseDeg(13.0, 8.5, 0.0);
    public Pose pick4EndLR = poseDeg(FIELD_SIZE - pick4EndLB.getX(), pick4EndLB.getY(), 0.0);

    // ===== Goal / Obelisk =====
    public Pose goalBlueAnchor = poseDeg(0.0, 144.0, 90.0);
    public Pose goalRedAnchor = mirrorBluePose(goalBlueAnchor);
    public Pose obeliskB = poseDeg(72.0, 144.0, 90.0);
    public Pose obeliskR = mirrorBluePose(obeliskB);

    public Pose getGoalPose(Alliance alliance, Range range, boolean preloadComplete) {
        Pose anchor = (alliance == Alliance.BLUE) ? goalBlueAnchor : goalRedAnchor;
        double dx = getGoalDxOffset(alliance, range, preloadComplete);
        return poseDeg(anchor.getX() + dx, anchor.getY(), 90.0);
    }

    private double getGoalDxOffset(Alliance alliance, Range range, boolean preloadComplete) {
        return 0.0;
    }

    public Pose getObeliskPose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? obeliskB : obeliskR;
    }

    public Pose findStartPose(Alliance a, Range r) {
        switch (a) {
            case BLUE:
                return (r == Range.CLOSE_RANGE) ? blueCloseStartPose : blueFarStartPose;
            case RED:
                return (r == Range.CLOSE_RANGE) ? redCloseStartPose : redFarStartPose;
            default:
                return new Pose(0, 0, 0);
        }
    }

    public final Pose[] load = new Pose[2];
    public final Pose[][] score = new Pose[2][2];
    public final Pose[][] pickupStartByRow = new Pose[2][5];
    public final Pose[][] pickupEndByRow = new Pose[2][5];
    public final Pose[][] leave = new Pose[2][2];
    public final Pose[][] releaseGoTo = new Pose[2][2];
    public final Pose[][] releaseComplete = new Pose[2][2];

    public AutoPoses() {
        setLoad(Alliance.BLUE, loadB);
        setLoad(Alliance.RED, loadR);

        setScore(Alliance.BLUE, Range.CLOSE_RANGE, scoreCB);
        setScore(Alliance.BLUE, Range.LONG_RANGE, scoreLB);
        setScore(Alliance.RED, Range.CLOSE_RANGE, scoreCR);
        setScore(Alliance.RED, Range.LONG_RANGE, scoreLR);

        setPickupStartAbsolute(Alliance.BLUE, 1, pick1StartCB);
        setPickupStartAbsolute(Alliance.BLUE, 2, pick2StartCB);
        setPickupStartAbsolute(Alliance.BLUE, 3, pick3StartCB);
        setPickupStartAbsolute(Alliance.BLUE, 4, pick4StartLB);
        setPickupStartAbsolute(Alliance.RED, 1, pick1StartCR);
        setPickupStartAbsolute(Alliance.RED, 2, pick2StartCR);
        setPickupStartAbsolute(Alliance.RED, 3, pick3StartCR);
        setPickupStartAbsolute(Alliance.RED, 4, pick4StartLR);

        setPickupEndAbsolute(Alliance.BLUE, 1, pick1EndCB);
        setPickupEndAbsolute(Alliance.BLUE, 2, pick2EndCB);
        setPickupEndAbsolute(Alliance.BLUE, 3, pick3EndCB);
        setPickupEndAbsolute(Alliance.BLUE, 4, pick4EndLB);
        setPickupEndAbsolute(Alliance.RED, 1, pick1EndCR);
        setPickupEndAbsolute(Alliance.RED, 2, pick2EndCR);
        setPickupEndAbsolute(Alliance.RED, 3, pick3EndCR);
        setPickupEndAbsolute(Alliance.RED, 4, pick4EndLR);

        setLeave(Alliance.BLUE, Range.CLOSE_RANGE, finalShootCloseB);
        setLeave(Alliance.BLUE, Range.LONG_RANGE, leaveLB);
        setLeave(Alliance.RED, Range.CLOSE_RANGE, finalShootCloseR);
        setLeave(Alliance.RED, Range.LONG_RANGE, leaveLR);

        setReleaseGoTo(Alliance.BLUE, Range.CLOSE_RANGE, releaseGoToCloseB);
        setReleaseGoTo(Alliance.BLUE, Range.LONG_RANGE, releaseGoToCloseB);
        setReleaseGoTo(Alliance.RED, Range.CLOSE_RANGE, releaseGoToCloseR);
        setReleaseGoTo(Alliance.RED, Range.LONG_RANGE, releaseGoToCloseR);

        setReleaseComplete(Alliance.BLUE, Range.CLOSE_RANGE, releaseCompleteCloseB);
        setReleaseComplete(Alliance.BLUE, Range.LONG_RANGE, releaseCompleteCloseB);
        setReleaseComplete(Alliance.RED, Range.CLOSE_RANGE, releaseCompleteCloseR);
        setReleaseComplete(Alliance.RED, Range.LONG_RANGE, releaseCompleteCloseR);
    }

    private void setLoad(Alliance a, Pose pose) {
        load[a.ordinal()] = pose;
    }

    private void setScore(Alliance a, Range r, Pose pose) {
        score[a.ordinal()][r.ordinal()] = pose;
    }

    private void setPickupStartAbsolute(Alliance a, int absoluteRow, Pose pose) {
        if (absoluteRow >= 1 && absoluteRow <= 4) {
            pickupStartByRow[a.ordinal()][absoluteRow] = pose;
        }
    }

    private void setPickupEndAbsolute(Alliance a, int absoluteRow, Pose pose) {
        if (absoluteRow >= 1 && absoluteRow <= 4) {
            pickupEndByRow[a.ordinal()][absoluteRow] = pose;
        }
    }

    private void setLeave(Alliance a, Range r, Pose pose) {
        leave[a.ordinal()][r.ordinal()] = pose;
    }

    private void setReleaseGoTo(Alliance a, Range r, Pose pose) {
        releaseGoTo[a.ordinal()][r.ordinal()] = pose;
    }

    private void setReleaseComplete(Alliance a, Range r, Pose pose) {
        releaseComplete[a.ordinal()][r.ordinal()] = pose;
    }

    public Pose getLoad(Alliance a) {
        return load[a.ordinal()];
    }

    public Pose getScore(Alliance a, Range r) {
        return score[a.ordinal()][r.ordinal()];
    }

    public Pose getFinalShootClose(Alliance a) {
        return (a == Alliance.BLUE) ? finalShootCloseB : finalShootCloseR;
    }

    public Pose getRow2ShootClose(Alliance a) {
        return (a == Alliance.BLUE) ? row2ShootCloseB : row2ShootCloseR;
    }

    public Pose getPickupStart(Alliance a, int absoluteRow) {
        int clamped = Math.max(1, Math.min(absoluteRow, 4));
        return pickupStartByRow[a.ordinal()][clamped];
    }

    public Pose getPickupStart(Alliance a, Range r, int absoluteRow) {
        int clamped = Math.max(1, Math.min(absoluteRow, 4));
        switch (clamped) {
            case 1:
                return (a == Alliance.BLUE) ? pick1StartCB : pick1StartCR;
            case 2:
                if (r == Range.LONG_RANGE) {
                    return (a == Alliance.BLUE) ? pick2StartLB : pick2StartLR;
                }
                return (a == Alliance.BLUE) ? pick2StartCB : pick2StartCR;
            case 3:
                if (r == Range.LONG_RANGE) {
                    return (a == Alliance.BLUE) ? pick3StartLB : pick3StartLR;
                }
                return (a == Alliance.BLUE) ? pick3StartCB : pick3StartCR;
            case 4:
            default:
                return (a == Alliance.BLUE) ? pick4StartLB : pick4StartLR;
        }
    }

    public Pose getPickupEnd(Alliance a, int absoluteRow) {
        int clamped = Math.max(1, Math.min(absoluteRow, 4));
        return pickupEndByRow[a.ordinal()][clamped];
    }

    public Pose getPickupEnd(Alliance a, Range r, int absoluteRow) {
        int clamped = Math.max(1, Math.min(absoluteRow, 4));
        switch (clamped) {
            case 1:
                return (a == Alliance.BLUE) ? pick1EndCB : pick1EndCR;
            case 2:
                if (r == Range.LONG_RANGE) {
                    return (a == Alliance.BLUE) ? pick2EndLB : pick2EndLR;
                }
                return (a == Alliance.BLUE) ? pick2EndCB : pick2EndCR;
            case 3:
                if (r == Range.LONG_RANGE) {
                    return (a == Alliance.BLUE) ? pick3EndLB : pick3EndLR;
                }
                return (a == Alliance.BLUE) ? pick3EndCB : pick3EndCR;
            case 4:
            default:
                return (a == Alliance.BLUE) ? pick4EndLB : pick4EndLR;
        }
    }

    public Pose getPickupRow4Step1(Alliance a) {
        return (a == Alliance.BLUE) ? pick4Step1LB : pick4Step1LR;
    }

    public Pose getFarPickupZone(Alliance a) {
        return (a == Alliance.BLUE) ? farPickupZoneB : farPickupZoneR;
    }

    public Pose getFarPickupZoneControl(Alliance a) {
        return (a == Alliance.BLUE) ? farPickupZoneControlB : farPickupZoneControlR;
    }

    public Pose getRow4GoToPickup(Alliance a) {
        return (a == Alliance.BLUE) ? row4GoToPickupB : row4GoToPickupR;
    }

    public Pose getRow4IntermediatePickup(Alliance a) {
        return (a == Alliance.BLUE) ? row4IntermediatePickupB : row4IntermediatePickupR;
    }

    public Pose getRow4CompletePickupControl(Alliance a) {
        return (a == Alliance.BLUE) ? row4CompletePickupControlB : row4CompletePickupControlR;
    }

    public Pose getRow4CompletePickup(Alliance a) {
        return (a == Alliance.BLUE) ? row4CompletePickupB : row4CompletePickupR;
    }

    public Pose getCloseLoopPickup(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopPickupB : closeLoopPickupR;
    }

    public Pose getCloseLoopPickupControl(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopPickupControlB : closeLoopPickupControlR;
    }

    public Pose getCloseLoopGoToScoreControl(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopGoToScoreControlB : closeLoopGoToScoreControlR;
    }

    public Pose getRow2GoToScoreControl(Alliance a) {
        return (a == Alliance.BLUE) ? row2GoToScoreControlB : row2GoToScoreControlR;
    }

    public Pose getCloseLoopSharedControl(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopSharedControlB : closeLoopSharedControlR;
    }

    public Pose getCloseLoopCompletePickup(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopCompletePickupB : closeLoopCompletePickupR;
    }

    public Pose getCloseLoopCompletePickupControl(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopCompletePickupControlB : closeLoopCompletePickupControlR;
    }

    public Pose getCloseLoopFinalShootControl(Alliance a) {
        return (a == Alliance.BLUE) ? closeLoopFinalShootControlB : closeLoopFinalShootControlR;
    }

    public Pose getLeave(Alliance a, Range r) {
        if (ReturnToStart) {
            if (a == Alliance.BLUE) {
                return (r == Range.CLOSE_RANGE) ? blueCloseStartPose : blueFarStartPose;
            }
            return (r == Range.CLOSE_RANGE) ? redCloseStartPose : redFarStartPose;
        }
        return leave[a.ordinal()][r.ordinal()];
    }

    public Pose getReleaseGoTo(Alliance a, Range r) {
        return releaseGoTo[a.ordinal()][r.ordinal()];
    }

    public Pose getReleaseComplete(Alliance a, Range r) {
        return releaseComplete[a.ordinal()][r.ordinal()];
    }
}
