package org.firstinspires.ftc.teamcode.config.utility;

public final class GlobalVariables {

    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum MotifPattern {
        PPG,
        GPP,
        PGP
    }

    private static AllianceColor allianceColor = AllianceColor.BLUE;
    private static MotifPattern motif = MotifPattern.PPG;
    private static boolean autoFollowerValid = false;

    private GlobalVariables() {
    }

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor color) {
        if (color != null && allianceColor != color) {
            allianceColor = color;
            // Changing alliance invalidates auto->teleop pose handoff assumptions.
            autoFollowerValid = false;
        }
    }

    public static boolean isBlueAlliance() {
        return allianceColor == AllianceColor.BLUE;
    }

    public static boolean isRedAlliance() {
        return allianceColor == AllianceColor.RED;
    }

    public static String getAllianceColorName() {
        return allianceColor.name().toLowerCase();
    }

    public static MotifPattern getMotif() {
        return motif;
    }

    public static void setMotif(MotifPattern nextMotif) {
        if (nextMotif != null) {
            motif = nextMotif;
        }
    }

    public static void toggleAlliance() {
        setAllianceColor(isBlueAlliance() ? AllianceColor.RED : AllianceColor.BLUE);
    }

    public static void nextMotif() {
        if (motif == MotifPattern.PPG) {
            motif = MotifPattern.GPP;
        } else if (motif == MotifPattern.GPP) {
            motif = MotifPattern.PGP;
        } else {
            motif = MotifPattern.PPG;
        }
    }

    public static boolean isAutoFollowerValid() {
        return autoFollowerValid;
    }

    public static void setAutoFollowerValid(boolean valid) {
        autoFollowerValid = valid;
    }

    public static void resetForMatch() {
        allianceColor = AllianceColor.BLUE;
        motif = MotifPattern.PPG;
        autoFollowerValid = false;
    }
}
