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

    private static AllianceColor allianceColor = AllianceColor.RED;
    private static MotifPattern motif = MotifPattern.PPG;
    private static boolean autoFollowerValid = false;

    private GlobalVariables() {
    }

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor color) {
        if (color != null) {
            allianceColor = color;
        }
    }

    public static void setAllianceColor(String color) {
        if (color == null) return;
        if (color.equalsIgnoreCase("blue")) {
            allianceColor = AllianceColor.BLUE;
        } else if (color.equalsIgnoreCase("red")) {
            allianceColor = AllianceColor.RED;
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

    public static void setMotif(String motifName) {
        if (motifName == null) return;
        if (motifName.equalsIgnoreCase("PPG")) {
            motif = MotifPattern.PPG;
        } else if (motifName.equalsIgnoreCase("GPP")) {
            motif = MotifPattern.GPP;
        } else if (motifName.equalsIgnoreCase("PGP")) {
            motif = MotifPattern.PGP;
        }
    }

    public static String getMotifName() {
        return motif.name();
    }

    public static boolean isAutoFollowerValid() {
        return autoFollowerValid;
    }

    public static void setAutoFollowerValid(boolean valid) {
        autoFollowerValid = valid;
    }

    public static void resetForMatch() {
        allianceColor = AllianceColor.RED;
        motif = MotifPattern.PPG;
        autoFollowerValid = false;
    }
}
