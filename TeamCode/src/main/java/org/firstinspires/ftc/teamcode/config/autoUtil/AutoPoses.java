package org.firstinspires.ftc.teamcode.config.autoUtil;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

public class AutoPoses {
    public static boolean ReturnToStart = true;
    private static final double FIELD_SIZE = 144.0;
    private static final double ROBOT_WIDTH = 17.5;
    private static final double ROBOT_LENGTH = 18;
    private static final double INTAKE_START_X = 42;
    private static final double INTAKE_END_X = 24;
    private static final double GOAL_Y = 144.0;
    private static final double GOAL_HEADING_DEG = 90.0;
    private static final double OBELISK_X = 72.0;
    private static final double OBELISK_Y = 144.0;
    private static final double OBELISK_HEADING_DEG = 90.0;

    private Pose poseDeg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    public Pose Mirror(Pose bluePose) {
        double x = FIELD_SIZE - bluePose.getX();
        double y = bluePose.getY();
        double h = normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(x, y, h);
    }

    private Pose offsetPose(Pose basePose, double dx, double dy, double dHeadingDeg) {
        double x = basePose.getX() + dx;
        double y = basePose.getY() + dy;
        double h = Math.toDegrees(basePose.getHeading()) + dHeadingDeg;
        return poseDeg(x, y, h);
    }

    public Pose baseToBlue(Pose basePose) {
        return basePose;
    }

    public Pose baseToBlue(Pose basePose, double dx, double dy, double dHeadingDeg) {
        return offsetPose(basePose, dx, dy, dHeadingDeg);
    }

    public Pose baseToRed(Pose basePose) {
        return Mirror(basePose);
    }

    public Pose baseToRed(Pose basePose, double dx, double dy, double dHeadingDeg) {
        return Mirror(offsetPose(basePose, dx, dy, dHeadingDeg));
    }

    //------------------ BASE POSES (BLUE SIDE COORDS) ------------------
    public final Base base = new Base();

    public class Base {
        private final double intakeHeadingDeg = 180.0;

        // Starting Poses
        public final Pose closeStart = poseDeg(23.567, 123.916 + 1, 216 + 18 - 2);
        public final Pose farStart = poseDeg(48 + ROBOT_LENGTH / 2, ROBOT_WIDTH / 2, 0);

        // ========================
        // SCORE POSES
        // ========================
        public final Pose scoreClose = poseDeg(48, 96, Math.toDegrees(closeStart.getHeading()));
        public final Pose scoreLong = poseDeg(54, 16, Math.toDegrees(farStart.getHeading()));

        //TODO add a secondary score pose if need to save time (Closer to middle row, top of close triangle)

        // ========================
        // LOAD POSES
        // ========================
        public final Pose load = poseDeg(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, 90);

        // ========================
        // GOAL / OBELISK POSES
        // ========================
        public final Pose goalBlueAnchor = poseDeg(0, GOAL_Y, GOAL_HEADING_DEG);
        public final Pose goalRedAnchor = poseDeg(FIELD_SIZE, GOAL_Y, GOAL_HEADING_DEG);
        public final Pose obelisk = poseDeg(OBELISK_X, OBELISK_Y, OBELISK_HEADING_DEG);

        // ========================
        // RELEASE (lever) POSES
        // ========================
        public final Pose releaseGoToClose = poseDeg(20, 76, 0);
        public final Pose releaseCompleteClose = poseDeg(14.25, 76, 0);

        // ========================
        // LEAVE POSES
        // ========================
        // Close-side park is nearer the scoring area; long-side park is farther.
        public final Pose leaveClose = poseDeg(36, 84, intakeHeadingDeg);
        public final Pose leaveLong = poseDeg(15, 15, intakeHeadingDeg);

        // ========================
        // PICKUP START POSES
        // ========================
        // Row 1 (close side)
        public final Pose pick1StartClose = poseDeg(INTAKE_START_X, 84, intakeHeadingDeg);

        // Row 2
        public final Pose pick2StartLong = poseDeg(INTAKE_START_X, 36, intakeHeadingDeg);
        public final Pose pick2StartClose = poseDeg(INTAKE_START_X, 60, intakeHeadingDeg);

        // Row 3
        public final Pose pick3StartLong = poseDeg(INTAKE_START_X, 60, intakeHeadingDeg);
        public final Pose pick3StartClose = poseDeg(INTAKE_START_X, 36, intakeHeadingDeg);

        // Row 4 (long side)
        public final Pose pick4StartLong = poseDeg(20, 12, intakeHeadingDeg);
        public final Pose farPickupZone = pick4StartLong;

        // ========================
        // PICKUP END POSES
        // ========================
        // Row 1 (close side)
        public final Pose pick1EndClose = poseDeg(INTAKE_END_X, 84, intakeHeadingDeg);

        // Row 2
        public final Pose pick2EndLong = poseDeg(INTAKE_END_X, 36, intakeHeadingDeg);
        public final Pose pick2EndClose = poseDeg(INTAKE_END_X, 60, intakeHeadingDeg);

        // Row 3
        public final Pose pick3EndLong = poseDeg(INTAKE_END_X, 60, intakeHeadingDeg);
        public final Pose pick3EndClose = poseDeg(INTAKE_END_X, 36, intakeHeadingDeg);

        // Row 4 (long side)
        public final Pose pick4EndLong = poseDeg(12, 12, intakeHeadingDeg);
    }

    // ------------------------
    // DERIVED POSES (BLUE/RED)
    // ------------------------
    public Pose blueCloseStartPose = baseToBlue(base.closeStart);
    public Pose blueFarStartPose = baseToBlue(base.farStart);
    public Pose redCloseStartPose = poseDeg(122, 117, 126);
    public Pose redFarStartPose = baseToRed(base.farStart);

    public Pose scoreCB = baseToBlue(base.scoreClose);
    public Pose scoreLB = baseToBlue(base.scoreLong);
    public Pose scoreCR = poseDeg(baseToRed(base.scoreClose).getX(), baseToRed(base.scoreClose).getY(),
            Math.toDegrees(redCloseStartPose.getHeading()));
    public Pose scoreLR = baseToRed(base.scoreLong);

    public Pose loadB = baseToBlue(base.load);
    public Pose loadR = baseToRed(base.load);

    public Pose releaseGoToCloseB = baseToBlue(base.releaseGoToClose);
    public Pose releaseCompleteCloseB = baseToBlue(base.releaseCompleteClose);
    public Pose releaseGoToCloseR = baseToRed(base.releaseGoToClose);
    public Pose releaseCompleteCloseR = baseToRed(base.releaseCompleteClose);

    public Pose leaveCB = baseToBlue(base.leaveClose);
    public Pose leaveLB = baseToBlue(base.leaveLong);
    public Pose leaveCR = baseToRed(base.leaveClose);
    public Pose leaveLR = baseToRed(base.leaveLong);

    public Pose pick1StartCB = baseToBlue(base.pick1StartClose);
    public Pose pick1StartCR = baseToRed(base.pick1StartClose);

    public Pose pick2StartLB = baseToBlue(base.pick2StartLong);
    public Pose pick2StartCB = baseToBlue(base.pick2StartClose);
    public Pose pick2StartCR = baseToRed(base.pick2StartClose);
    public Pose pick2StartLR = baseToRed(base.pick2StartLong);

    public Pose pick3StartLB = baseToBlue(base.pick3StartLong);
    public Pose pick3StartCB = baseToBlue(base.pick3StartClose);
    public Pose pick3StartCR = baseToRed(base.pick3StartClose);
    public Pose pick3StartLR = baseToRed(base.pick3StartLong);

    public Pose pick4StartLB = baseToBlue(base.pick4StartLong);
    public Pose pick4StartLR = baseToRed(base.pick4StartLong);
    public Pose farPickupZoneB = baseToBlue(base.farPickupZone);
    public Pose farPickupZoneR = baseToRed(base.farPickupZone);

    public Pose pick1EndCB = baseToBlue(base.pick1EndClose);
    public Pose pick1EndCR = baseToRed(base.pick1EndClose);

    public Pose pick2EndLB = baseToBlue(base.pick2EndLong);
    public Pose pick2EndCB = baseToBlue(base.pick2EndClose);
    public Pose pick2EndCR = baseToRed(base.pick2EndClose);
    public Pose pick2EndLR = baseToRed(base.pick2EndLong);

    public Pose pick3EndLB = baseToBlue(base.pick3EndLong);
    public Pose pick3EndCB = baseToBlue(base.pick3EndClose);
    public Pose pick3EndCR = baseToRed(base.pick3EndClose);
    public Pose pick3EndLR = baseToRed(base.pick3EndLong);

    public Pose pick4EndLB = baseToBlue(base.pick4EndLong);
    public Pose pick4EndLR = baseToRed(base.pick4EndLong);

    public Pose getGoalPose(Alliance alliance, Range range, boolean preloadComplete) {
        Pose anchor = (alliance == Alliance.BLUE) ? base.goalBlueAnchor : base.goalRedAnchor;
        double dx = getGoalDxOffset(alliance, range, preloadComplete);
        return poseDeg(anchor.getX() + dx, anchor.getY(), GOAL_HEADING_DEG);
    }

    private double getGoalDxOffset(Alliance alliance, Range range, boolean preloadComplete) {
        if (alliance == Alliance.BLUE) {
            if (range == Range.LONG_RANGE) {
                return preloadComplete ? 0 : 0; //Blue, long range
            }
            return preloadComplete ? 0 : 0; //Blue, close range
        }
        else if (alliance == Alliance.RED)
        {
            if (range == Range.LONG_RANGE) {
                return preloadComplete ? 0 : 0; //Red, long range
            }
            return preloadComplete ? 0.0 : 0.0; //Red, short range
        }
        return 0.0;
    }

    public Pose getObeliskPose(Alliance alliance) {
        return (alliance == Alliance.BLUE)
                ? baseToBlue(base.obelisk)
                : baseToRed(base.obelisk);
    }

    public Pose findStartPose(Alliance a, Range r) {
        switch (a) {
            case BLUE:
                return (r == Range.CLOSE_RANGE) ? blueCloseStartPose : blueFarStartPose;
            case RED:
                return (r == Range.CLOSE_RANGE) ? redCloseStartPose  : redFarStartPose;
            default:
                return new Pose(0,0,0);
        }
    }

     /* =========================
       STRUCTURED ARRAYS
       =========================
       load[alliance]
       score[alliance][range]
       pickupStartByRow[alliance][absoluteRow]
       pickupEndByRow[alliance][absoluteRow]
       ========================= */

    public final Pose[] load = new Pose[2];                    // [Alliance]
    public final Pose[][] score = new Pose[2][2];              // [Alliance][Range]
    public final Pose[][] pickupStartByRow = new Pose[2][5];   // [Alliance][AbsoluteRow 1..4]
    public final Pose[][] pickupEndByRow = new Pose[2][5];     // [Alliance][AbsoluteRow 1..4]
    public final Pose[][] leave = new Pose[2][2];              // [Alliance][Range]
    public final Pose[][] releaseGoTo = new Pose[2][2];        // [Alliance][Range]
    public final Pose[][] releaseComplete = new Pose[2][2];    // [Alliance][Range]


    /* =========================
       FILL ARRAYS
       ========================= */
    public AutoPoses() {
        // --- Load ---
        setLoad(Alliance.BLUE, loadB);
        setLoad(Alliance.RED, loadR);

        // --- Score ---
        setScore(Alliance.BLUE, Range.CLOSE_RANGE, scoreCB);
        setScore(Alliance.BLUE, Range.LONG_RANGE, scoreLB);
        setScore(Alliance.RED, Range.CLOSE_RANGE, scoreCR);
        setScore(Alliance.RED, Range.LONG_RANGE, scoreLR);

        // --- Pickup absolute rows (1..4, closest-to-goal outward) ---
        // NOTE: Row 4 uses the existing load-side lane points.
        // Blue
        setPickupStartAbsolute(Alliance.BLUE, 1, pick1StartCB);
        setPickupStartAbsolute(Alliance.BLUE, 2, pick2StartCB);
        setPickupStartAbsolute(Alliance.BLUE, 3, pick3StartCB);
        setPickupStartAbsolute(Alliance.BLUE, 4, pick4StartLB);

        // Red
        setPickupStartAbsolute(Alliance.RED, 1, pick1StartCR);
        setPickupStartAbsolute(Alliance.RED, 2, pick2StartCR);
        setPickupStartAbsolute(Alliance.RED, 3, pick3StartCR);
        setPickupStartAbsolute(Alliance.RED, 4, pick4StartLR);

        // --- Pickup End absolute rows ---
        // Blue
        setPickupEndAbsolute(Alliance.BLUE, 1, pick1EndCB);
        setPickupEndAbsolute(Alliance.BLUE, 2, pick2EndCB);
        setPickupEndAbsolute(Alliance.BLUE, 3, pick3EndCB);
        setPickupEndAbsolute(Alliance.BLUE, 4, pick4EndLB);

        // Red
        setPickupEndAbsolute(Alliance.RED, 1, pick1EndCR);
        setPickupEndAbsolute(Alliance.RED, 2, pick2EndCR);
        setPickupEndAbsolute(Alliance.RED, 3, pick3EndCR);
        setPickupEndAbsolute(Alliance.RED, 4, pick4EndLR);

        // --- Leave ---
        setLeave(Alliance.BLUE, Range.CLOSE_RANGE, leaveCB);
        setLeave(Alliance.BLUE, Range.LONG_RANGE, leaveLB);
        setLeave(Alliance.RED, Range.CLOSE_RANGE, leaveCR);
        setLeave(Alliance.RED, Range.LONG_RANGE, leaveLR);

        // --- Release lever (only defined for close side; long side mirrors close defaults) ---
        setReleaseGoTo(Alliance.BLUE, Range.CLOSE_RANGE, releaseGoToCloseB);
        setReleaseGoTo(Alliance.BLUE, Range.LONG_RANGE, releaseGoToCloseB); // not used, but set for completeness
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

    /* =========================
       SIMPLE GETTERS
       ========================= */
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

    public Pose getPickupEnd(Alliance a, int absoluteRow) {
        int clamped = Math.max(1, Math.min(absoluteRow, 4));
        return pickupEndByRow[a.ordinal()][clamped];
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

    public Pose getReleaseGoTo(Alliance a, Range r) { return releaseGoTo[a.ordinal()][r.ordinal()]; }

    public Pose getReleaseComplete(Alliance a, Range r) { return releaseComplete[a.ordinal()][r.ordinal()]; }
}
