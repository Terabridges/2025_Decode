package org.firstinspires.ftc.teamcode.config.autoUtil;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.pedropathing.geometry.Pose;

public class AutoPoses {

    // ---- index helpers ----
    private static int rIdx(Range r) {
        return (r == Range.CLOSE_RANGE) ? 0 : 1;   // 0 = close, 1 = long
    }
    private static int mIdx(Mode m) {
        switch (m) {
            case ONE_ROW:   return 0;
            case TWO_ROW:   return 1;
            case THREE_ROW: return 2;
            case FOUR_ROW:  return 3;
            default:        return 0; // or throw if pickup is not used for this mode
        }
    }

    public Pose Mirror(Pose bluePose) {
        double x = 144 - bluePose.getX();
        double y = bluePose.getY();
        double h = normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(x, y, h);
    }

    //------------------ POSES ------------------

    //Starting Poses
    public Pose blueCloseStartPose = new Pose();
    public Pose blueFarStartPose = new Pose();
    public Pose redCloseStartPose = new Pose();
    public Pose redFarStartPose = new Pose();

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
    public Pose scoreCB = new Pose();
    public Pose scoreLB = new Pose();

    // Red = mirror of Blue
    public Pose scoreCR = Mirror(scoreCB);
    public Pose scoreLR = Mirror(scoreLB);

    // ========================
    // LOAD POSES
    // ========================
    public Pose loadB = new Pose();
    public Pose loadR = Mirror(loadB);

    // ========================
    // PICKUP START POSES
    // ========================

    // Row 1
    public Pose pick1StartCB = new Pose();
    public Pose pick1StartLB = new Pose();
    public Pose pick1StartCR = Mirror(pick1StartCB);
    public Pose pick1StartLR = Mirror(pick1StartLB);

    // Row 2
    public Pose pick2StartCB = new Pose();
    public Pose pick2StartLB = new Pose();
    public Pose pick2StartCR = Mirror(pick2StartCB);
    public Pose pick2StartLR = Mirror(pick2StartLB);

    // Row 3
    public Pose pick3StartCB = new Pose();
    public Pose pick3StartLB = new Pose();
    public Pose pick3StartCR = Mirror(pick3StartCB);
    public Pose pick3StartLR = Mirror(pick3StartLB);

    // Row 4
    public Pose pick4StartCB = new Pose();
    public Pose pick4StartLB = new Pose();
    public Pose pick4StartCR = Mirror(pick4StartCB);
    public Pose pick4StartLR = Mirror(pick4StartLB);

    // ========================
    // PICKUP END POSES
    // ========================

    // Row 1
    public Pose pick1EndCB = new Pose();
    public Pose pick1EndLB = new Pose();
    public Pose pick1EndCR = Mirror(pick1EndCB);
    public Pose pick1EndLR = Mirror(pick1EndLB);

    // Row 2
    public Pose pick2EndCB = new Pose();
    public Pose pick2EndLB = new Pose();
    public Pose pick2EndCR = Mirror(pick2EndCB);
    public Pose pick2EndLR = Mirror(pick2EndLB);

    // Row 3
    public Pose pick3EndCB = new Pose();
    public Pose pick3EndLB = new Pose();
    public Pose pick3EndCR = Mirror(pick3EndCB);
    public Pose pick3EndLR = Mirror(pick3EndLB);

    // Row 4
    public Pose pick4EndCB = new Pose();
    public Pose pick4EndLB = new Pose();
    public Pose pick4EndCR = Mirror(pick4EndCB);
    public Pose pick4EndLR = Mirror(pick4EndLB);

     /* =========================
       STRUCTURED ARRAYS
       =========================
       load[alliance]
       score[alliance][range]
       pickupStart[alliance][range][mode]
       pickupEnd[alliance][range][mode]
       ========================= */

    public Pose[] load = new Pose[2];                 // [Alliance]
    public Pose[][] score = new Pose[2][2];           // [Alliance][Range]
    public Pose[][][] pickupStart = new Pose[2][2][4]; // [Alliance][Range][Mode]
    public Pose[][][] pickupEnd = new Pose[2][2][4];   // [Alliance][Range][Mode]


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

        // --- Pickup Start (rows = modes 0..3) ---
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

    public Pose getPickupStart(Alliance a, Range r, Mode m) {
        return pickupStart[a.ordinal()][r.ordinal()][m.ordinal()];
    }

    public Pose getPickupEnd(Alliance a, Range r, Mode m) {
        return pickupEnd[a.ordinal()][r.ordinal()][m.ordinal()];
    }
}
