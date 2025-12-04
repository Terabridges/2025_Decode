package org.firstinspires.ftc.teamcode.config.autoUtil;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

public class AutoPoses {

    // ---- index helpers ----
    private static int rIdx(Range r) {
        return (r == Range.CLOSE_RANGE) ? 0 : 1;   // 0 = close, 1 = long
    }

    private Pose poseDeg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    public Pose Mirror(Pose bluePose) {
        double x = 144 - bluePose.getX();
        double y = bluePose.getY();
        double h = normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(x, y, h);
    }

    //------------------ POSES ------------------

    double robotWidth = 14.5;
    double robotLength = 15.25;

    //Starting Poses
    public Pose blueCloseStartPose = poseDeg(48 + robotWidth/2, robotLength/2, 180); //TODO use actual pose
    public Pose blueFarStartPose = poseDeg(48 + robotWidth/2, robotLength/2, 180);
    public Pose redCloseStartPose = Mirror(blueCloseStartPose);
    public Pose redFarStartPose = Mirror(blueFarStartPose);

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

    // ========================
    // SCORE POSES
    // ========================
    public Pose scoreCB = poseDeg(48, 96, 80);
    public Pose scoreLB = poseDeg(48 + robotWidth/2, 16, 210);
    public Pose scoreCR = Mirror(scoreCB);
    public Pose scoreLR = Mirror(scoreLB);

    //TODO add a secondary score pose if need to save time (Closer to middle row, top of close triangle)

    // ========================
    // LOAD POSES
    // ========================
    public Pose loadB = poseDeg(robotWidth/2, robotLength/2, 90);
    public Pose loadR = Mirror(loadB);

    // ========================
    // RELEASE (lever) POSES
    // ========================

    public Pose releaseGoToCloseB = poseDeg(32, 70, 80);
    public Pose releaseCompleteCloseB = poseDeg(28, 70, 210);
    public Pose releaseGoToCloseR = Mirror(releaseGoToCloseB);
    public Pose releaseCompleteCloseR = Mirror(releaseCompleteCloseB);

    // ========================
    // LEAVE POSES
    // ========================
    // Close-side park is nearer the scoring area; long-side park is farther.
    public Pose leaveCB = poseDeg(36, 84, 90);
    public Pose leaveLB = poseDeg(48, 36, 90);
    public Pose leaveCR = Mirror(leaveCB);
    public Pose leaveLR = Mirror(leaveLB);

    // ========================
    // PICKUP START POSES
    // ========================
    double intakeStart = 43.5;
    double offset = -3;

    // Row 1
    public Pose pick1StartLB = poseDeg(24, 16 + offset, 180); //TODO fix this
    public Pose pick1StartCB = poseDeg(intakeStart, 84, 180);
    public Pose pick1StartCR = Mirror(pick1StartCB);
    public Pose pick1StartLR = Mirror(pick1StartLB);

    // Row 2
    public Pose pick2StartLB = poseDeg(intakeStart, 36 + offset, 180);
    public Pose pick2StartCB = poseDeg(intakeStart, 60 + offset, 180);
    public Pose pick2StartCR = Mirror(pick2StartCB);
    public Pose pick2StartLR = Mirror(pick2StartLB);

    // Row 3
    public Pose pick3StartLB = poseDeg(intakeStart, 60 + offset, 180);
    public Pose pick3StartCB = poseDeg(intakeStart, 36 + offset, 180);
    public Pose pick3StartCR = Mirror(pick3StartCB);
    public Pose pick3StartLR = Mirror(pick3StartLB);

    // Row 4
    public Pose pick4StartLB = poseDeg(intakeStart, 84, 180);
    public Pose pick4StartCB = poseDeg(24, 16 + offset, 180); //TODO fix this
    public Pose pick4StartCR = Mirror(pick4StartCB);
    public Pose pick4StartLR = Mirror(pick4StartLB);

    // ========================
    // PICKUP END POSES
    // ========================
    double intakeEnd = 24.5;

    // Row 1
    public Pose pick1EndLB = poseDeg(16, 16 + offset, 180); //TODO fix this
    public Pose pick1EndCB = poseDeg(intakeEnd, 84, 180);
    public Pose pick1EndCR = Mirror(pick1EndCB);
    public Pose pick1EndLR = Mirror(pick1EndLB);

    // Row 2
    public Pose pick2EndLB = poseDeg(intakeEnd, 36 + offset, 180);
    public Pose pick2EndCB = poseDeg(intakeEnd, 60 + offset, 180);
    public Pose pick2EndCR = Mirror(pick2EndCB);
    public Pose pick2EndLR = Mirror(pick2EndLB);

    // Row 3
    public Pose pick3EndLB = poseDeg(intakeEnd, 60 + offset, 180);
    public Pose pick3EndCB = poseDeg(intakeEnd, 36 + offset, 180);
    public Pose pick3EndCR = Mirror(pick3EndCB);
    public Pose pick3EndLR = Mirror(pick3EndLB);

    // Row 4
    public Pose pick4EndLB = poseDeg(intakeEnd, 84, 180);
    public Pose pick4EndCB = poseDeg(16, 16 + offset, 180); //TODO fix this
    public Pose pick4EndCR = Mirror(pick4EndCB);
    public Pose pick4EndLR = Mirror(pick4EndLB);

     /* =========================
       STRUCTURED ARRAYS
       =========================
       load[alliance]
       score[alliance][range]
       pickupStart[alliance][range][rowIndex]
       pickupEnd[alliance][range][rowIndex]
       ========================= */

    public Pose[] load = new Pose[2];                  // [Alliance]
    public Pose[][] score = new Pose[2][2];            // [Alliance][Range]
    public Pose[][][] pickupStart = new Pose[2][2][4]; // [Alliance][Range][RowIndex]
    public Pose[][][] pickupEnd = new Pose[2][2][4];   // [Alliance][Range][RowIndex]
    public Pose[][] leave = new Pose[2][2];            // [Alliance][Range]
    public Pose[][] releaseGoTo = new Pose[2][2];      // [Alliance][Range]
    public Pose[][] releaseComplete = new Pose[2][2];  // [Alliance][Range]


    /* =========================
       FILL ARRAYS
       ========================= */
    public AutoPoses() {
        // --- Load ---
        load[Alliance.BLUE.ordinal()] = loadB;
        load[Alliance.RED.ordinal()]  = loadR;

        // --- Score ---
        score[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()] = scoreCB;
        score[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()]  = scoreLB;
        score[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()]  = scoreCR;
        score[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()]   = scoreLR;

        // --- Pickup Start (rows = index 0..3) ---
        // Blue
        pickupStart[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][0] = pick1StartCB;
        pickupStart[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][0]  = pick1StartLB;
        pickupStart[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][1] = pick2StartCB;
        pickupStart[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][1]  = pick2StartLB;
        pickupStart[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][2] = pick3StartCB;
        pickupStart[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][2]  = pick3StartLB;
        pickupStart[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][3] = pick4StartCB;
        pickupStart[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][3]  = pick4StartLB;

        // Red
        pickupStart[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][0] = pick1StartCR;
        pickupStart[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][0]  = pick1StartLR;
        pickupStart[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][1] = pick2StartCR;
        pickupStart[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][1]  = pick2StartLR;
        pickupStart[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][2] = pick3StartCR;
        pickupStart[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][2]  = pick3StartLR;
        pickupStart[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][3] = pick4StartCR;
        pickupStart[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][3]  = pick4StartLR;

        // --- Pickup End ---
        // Blue
        pickupEnd[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][0] = pick1EndCB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][0]  = pick1EndLB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][1] = pick2EndCB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][1]  = pick2EndLB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][2] = pick3EndCB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][2]  = pick3EndLB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()][3] = pick4EndCB;
        pickupEnd[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()][3]  = pick4EndLB;

        // Red
        pickupEnd[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][0] = pick1EndCR;
        pickupEnd[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][0]  = pick1EndLR;
        pickupEnd[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][1] = pick2EndCR;
        pickupEnd[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][1]  = pick2EndLR;
        pickupEnd[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][2] = pick3EndCR;
        pickupEnd[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][2]  = pick3EndLR;
        pickupEnd[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()][3] = pick4EndCR;
        pickupEnd[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()][3]  = pick4EndLR;

        // --- Leave ---
        leave[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()] = leaveCB;
        leave[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()]  = leaveLB;
        leave[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()]  = leaveCR;
        leave[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()]   = leaveLR;

        // --- Release lever (only defined for close side; long side mirrors close defaults) ---
        releaseGoTo[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()] = releaseGoToCloseB;
        releaseGoTo[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()]  = releaseGoToCloseB; // not used, but set for completeness
        releaseGoTo[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()]  = releaseGoToCloseR;
        releaseGoTo[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()]   = releaseGoToCloseR;

        releaseComplete[Alliance.BLUE.ordinal()][Range.CLOSE_RANGE.ordinal()] = releaseCompleteCloseB;
        releaseComplete[Alliance.BLUE.ordinal()][Range.LONG_RANGE.ordinal()]  = releaseCompleteCloseB;
        releaseComplete[Alliance.RED.ordinal()][Range.CLOSE_RANGE.ordinal()]  = releaseCompleteCloseR;
        releaseComplete[Alliance.RED.ordinal()][Range.LONG_RANGE.ordinal()]   = releaseCompleteCloseR;
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

    /** Returns the score pose for a given shot index using the "closest point" plan. */
    public Pose getClosestScore(Alliance a, Range selectedRange, int shotIndex) {
        // shotIndex: 0 = preload, 1 = row1, 2 = row2, 3 = row3, 4 = row4
        boolean useClose;
        if (selectedRange == Range.LONG_RANGE) {
            // Preload, row1, row2 = long; row3+, close
            useClose = shotIndex >= 2;
        } else {
            // Preload, row1, row2 = close; row3+, long
            useClose = shotIndex <= 2;
        }
        Range r = useClose ? Range.CLOSE_RANGE : Range.LONG_RANGE;
        return getScore(a, r);
    }

    public Pose getPickupStart(Alliance a, Range r, int rowIndex) {
        return pickupStart[a.ordinal()][r.ordinal()][rowIndex];
    }

    public Pose getPickupEnd(Alliance a, Range r, int rowIndex) {
        return pickupEnd[a.ordinal()][r.ordinal()][rowIndex];
    }

    public Pose getLeave(Alliance a, Range r) {
        return leave[a.ordinal()][r.ordinal()];
    }

    public Pose getReleaseGoTo(Alliance a, Range r) { return releaseGoTo[a.ordinal()][r.ordinal()]; }

    public Pose getReleaseComplete(Alliance a, Range r) { return releaseComplete[a.ordinal()][r.ordinal()]; }
}
