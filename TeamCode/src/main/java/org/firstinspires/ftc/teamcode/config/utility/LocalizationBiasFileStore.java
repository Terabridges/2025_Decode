package org.firstinspires.ftc.teamcode.config.utility;

import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

public final class LocalizationBiasFileStore {

    private static final File STORE_FILE = new File("/sdcard/FIRST/TeraBridges/localization-bias.properties");
    private static GlobalVariables.AllianceColor lastLoadedAlliance = null;
    private static LoadResult lastLoadResult = new LoadResult(Bias.invalid(), false, "not_loaded");

    private LocalizationBiasFileStore() {
    }

    public static Bias captureBias(Pose2d referencePose, Pose2d measuredPose) {
        if (referencePose == null || measuredPose == null) {
            return Bias.invalid();
        }

        return new Bias(
                referencePose.getX() - measuredPose.getX(),
                referencePose.getY() - measuredPose.getY(),
                Math.toDegrees(wrapRad(
                        referencePose.getRotation().getRadians() - measuredPose.getRotation().getRadians()
                )),
                true
        );
    }

    public static Pose2d applyBias(Pose2d pose, Bias bias) {
        if (pose == null || bias == null || !bias.valid) {
            return null;
        }

        return new Pose2d(
                pose.getX() + bias.xMeters,
                pose.getY() + bias.yMeters,
                Rotation2d.fromRadians(wrapRad(
                        pose.getRotation().getRadians() + Math.toRadians(bias.headingDeg)
                ))
        );
    }

    public static void saveAllianceBias(GlobalVariables.AllianceColor allianceColor, Bias bias) throws IOException {
        if (allianceColor == null || bias == null || !bias.valid) {
            return;
        }

        Properties props = loadProperties();
        String prefix = keyPrefix(allianceColor);
        props.setProperty(prefix + ".valid", "true");
        props.setProperty(prefix + ".xMeters", Double.toString(bias.xMeters));
        props.setProperty(prefix + ".yMeters", Double.toString(bias.yMeters));
        props.setProperty(prefix + ".headingDeg", Double.toString(bias.headingDeg));

        File parent = STORE_FILE.getParentFile();
        if (parent != null && !parent.exists()) {
            parent.mkdirs();
        }

        FileOutputStream out = new FileOutputStream(STORE_FILE);
        try {
            props.store(out, "TeraBridges localization bias calibration");
        } finally {
            out.close();
        }
    }

    public static Bias loadAllianceBias(GlobalVariables.AllianceColor allianceColor) throws IOException {
        if (allianceColor == null || !STORE_FILE.exists()) {
            return Bias.invalid();
        }

        Properties props = loadProperties();
        String prefix = keyPrefix(allianceColor);
        boolean valid = Boolean.parseBoolean(props.getProperty(prefix + ".valid", "false"));
        if (!valid) {
            return Bias.invalid();
        }

        return new Bias(
                parseDouble(props.getProperty(prefix + ".xMeters"), 0.0),
                parseDouble(props.getProperty(prefix + ".yMeters"), 0.0),
                parseDouble(props.getProperty(prefix + ".headingDeg"), 0.0),
                true
        );
    }

    public static void applyBiasToCalculator(Bias bias) {
        if (bias == null || !bias.valid) {
            return;
        }
        LocalizationCandidateCalculator.mt1FieldOffsetXMeters = bias.xMeters;
        LocalizationCandidateCalculator.mt1FieldOffsetYMeters = bias.yMeters;
        LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg = bias.headingDeg;
    }

    public static void clearCalculatorBias() {
        LocalizationCandidateCalculator.mt1FieldOffsetXMeters = 0.0;
        LocalizationCandidateCalculator.mt1FieldOffsetYMeters = 0.0;
        LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg = 0.0;
    }

    public static LoadResult ensureAllianceBiasLoaded(GlobalVariables.AllianceColor allianceColor) {
        if (allianceColor == null) {
            clearCalculatorBias();
            lastLoadedAlliance = null;
            lastLoadResult = new LoadResult(Bias.invalid(), false, "missing_alliance");
            return lastLoadResult;
        }

        if (allianceColor == lastLoadedAlliance) {
            return lastLoadResult;
        }

        lastLoadedAlliance = allianceColor;
        try {
            Bias bias = loadAllianceBias(allianceColor);
            if (bias.valid) {
                applyBiasToCalculator(bias);
                lastLoadResult = new LoadResult(bias, true, "loaded");
            } else {
                clearCalculatorBias();
                lastLoadResult = new LoadResult(Bias.invalid(), false, "missing");
            }
        } catch (Exception e) {
            clearCalculatorBias();
            lastLoadResult = new LoadResult(Bias.invalid(), false, "error:" + e.getClass().getSimpleName());
        }

        return lastLoadResult;
    }

    public static Bias getCalculatorBias() {
        return new Bias(
                LocalizationCandidateCalculator.mt1FieldOffsetXMeters,
                LocalizationCandidateCalculator.mt1FieldOffsetYMeters,
                LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg,
                true
        );
    }

    public static String getStorePath() {
        return STORE_FILE.getAbsolutePath();
    }

    private static Properties loadProperties() throws IOException {
        Properties props = new Properties();
        if (!STORE_FILE.exists()) {
            return props;
        }

        FileInputStream in = new FileInputStream(STORE_FILE);
        try {
            props.load(in);
        } finally {
            in.close();
        }
        return props;
    }

    private static String keyPrefix(GlobalVariables.AllianceColor allianceColor) {
        return allianceColor.name().toLowerCase();
    }

    private static double parseDouble(String raw, double defaultValue) {
        if (raw == null || raw.trim().isEmpty()) {
            return defaultValue;
        }
        return Double.parseDouble(raw.trim());
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    public static final class Bias {
        public final double xMeters;
        public final double yMeters;
        public final double headingDeg;
        public final boolean valid;

        public Bias(double xMeters, double yMeters, double headingDeg, boolean valid) {
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.headingDeg = headingDeg;
            this.valid = valid;
        }

        public static Bias invalid() {
            return new Bias(0.0, 0.0, 0.0, false);
        }
    }

    public static final class LoadResult {
        public final Bias bias;
        public final boolean loadedFromFile;
        public final String status;

        public LoadResult(Bias bias, boolean loadedFromFile, String status) {
            this.bias = (bias != null) ? bias : Bias.invalid();
            this.loadedFromFile = loadedFromFile;
            this.status = (status != null) ? status : "unknown";
        }
    }
}
