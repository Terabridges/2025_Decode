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

    // ===== Start Poses =====
    public Pose blueCloseStartPose = poseDeg(23.567, 124.916, 232.0);
    public Pose blueFarStartPose = poseDeg(48.0 + ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0, 0.0);
    public Pose redCloseStartPose = poseDeg(120.433, 124.916, 128);
    public Pose redFarStartPose = poseDeg(FIELD_SIZE - (48.0 + ROBOT_LENGTH / 2.0), ROBOT_WIDTH / 2.0, 0.0);

    // ===== Score Poses =====
    public Pose scoreCB = poseDeg(48.0, 96.0, 232.0);
    public Pose scoreLB = poseDeg(54.0, 16.0, 0.0);
    public Pose scoreCR = poseDeg(96.0, 96.0, 128);
    public Pose scoreLR = poseDeg(90.0, 16.0, 0.0);

    // ===== Load Poses =====
    public Pose loadB = poseDeg(ROBOT_WIDTH / 2.0, ROBOT_LENGTH / 2.0, 0);
    public Pose loadR = poseDeg(FIELD_SIZE - ROBOT_WIDTH / 2.0, ROBOT_LENGTH / 2.0, 0);

    // ===== Release Poses =====
    public Pose releaseGoToCloseB = poseDeg(23.0, 74.0, 180.0);
    public Pose releaseCompleteCloseB = poseDeg(17.5, 74.0, 180.0);
    public Pose releaseGoToCloseR = poseDeg(121.0, 74.0, 180);
    public Pose releaseCompleteCloseR = poseDeg(126.5, 74.0, 180);

    // ===== Leave Poses =====
    public Pose leaveCB = poseDeg(36.0, 84.0, 180.0);
    public Pose leaveLB = poseDeg(15.0, 15.0, 0.0);
    public Pose leaveCR = poseDeg(108.0, 84.0, 180);
    public Pose leaveLR = poseDeg(129.0, 15.0, 0.0);

    // ===== Pickup Start Poses =====
    public Pose pick1StartCB = poseDeg(44.0, 84.0, 180.0);
    public Pose pick1StartCR = poseDeg(100.0, 84.0, 180.0);

    public Pose pick2StartLB = poseDeg(44.0, 60.0, 0.0);
    public Pose pick2StartCB = poseDeg(44.0, 60.0, 180.0);
    public Pose pick2StartCR = poseDeg(100.0, 60.0, 180.0);
    public Pose pick2StartLR = poseDeg(100.0, 60.0, 0);

    public Pose pick3StartLB = poseDeg(44.0, 36.0, 0.0);
    public Pose pick3StartCB = poseDeg(44.0, 36.0, 180.0);
    public Pose pick3StartCR = poseDeg(97.5, 36.0, 180.0);
    public Pose pick3StartLR = poseDeg(100.0, 36.0, 0);

    public Pose pick4StartLB = poseDeg(13.5, 16.0, 20.0);
    public Pose pick4StartLR = poseDeg(130.5, 16.0, 340.0);

    public Pose farPickupZoneB = poseDeg(10.0, 11.5, 0.0);
    public Pose farPickupZoneR = poseDeg(134.0, 11.5, 0.0);

    // ===== Pickup End Poses =====
    public Pose pick1EndCB = poseDeg(24.0, 84.0, 180.0);
    public Pose pick1EndCR = poseDeg(120.0, 84.0, 180.0);

    public Pose pick2EndLB = poseDeg(24.0, 60.0, 0.0);
    public Pose pick2EndCB = poseDeg(24.0, 60.0, 180.0);
    public Pose pick2EndCR = poseDeg(120.0, 60.0, 180.0);
    public Pose pick2EndLR = poseDeg(120.0, 60.0, 0);

    public Pose pick3EndLB = poseDeg(24.0, 36.0, 0.0);
    public Pose pick3EndCB = poseDeg(24.0, 36.0, 180.0);
    public Pose pick3EndCR = poseDeg(120.0, 36.0, 180.0);
    public Pose pick3EndLR = poseDeg(120.0, 36.0, 0);

    public Pose pick4EndLB = poseDeg(13.5, 11.25, 20.0);
    public Pose pick4EndLR = poseDeg(130.5, 11.25, 340.0);

    // ===== Goal / Obelisk =====
    public Pose goalBlueAnchor = poseDeg(0.0, 144.0, 90.0);
    public Pose goalRedAnchor = poseDeg(144.0, 144.0, 90.0);
    public Pose obeliskB = poseDeg(72.0, 144.0, 90.0);
    public Pose obeliskR = poseDeg(72.0, 144.0, 90.0);

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

        setLeave(Alliance.BLUE, Range.CLOSE_RANGE, leaveCB);
        setLeave(Alliance.BLUE, Range.LONG_RANGE, leaveLB);
        setLeave(Alliance.RED, Range.CLOSE_RANGE, leaveCR);
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

    public Pose getFarPickupZone(Alliance a) {
        return (a == Alliance.BLUE) ? farPickupZoneB : farPickupZoneR;
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
