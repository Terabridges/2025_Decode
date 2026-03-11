package org.firstinspires.ftc.teamcode.psikit;

import org.firstinspires.ftc.teamcode.config.utility.LocalizationCandidateCalculator;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGReplay;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public final class LocalizationCandidateReplayCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String OUTPUT_DIR_PROPERTY = "psikitReplayOutputDir";
    private static final String OUTPUT_NAME_PROPERTY = "psikitReplayOutputName";
    private static final String CAL_X_PROPERTY = "mt1CalibrationXMeters";
    private static final String CAL_Y_PROPERTY = "mt1CalibrationYMeters";
    private static final String CAL_HEADING_PROPERTY = "mt1CalibrationHeadingDeg";

    private LocalizationCandidateReplayCliMain() {
    }

    public static void main(String[] args) throws Exception {
        Path logPath = resolveRequiredPath(LOG_PATH_PROPERTY);
        Path outputDir = resolveOutputDir();
        String outputName = resolveOutputName(logPath);
        configureCalibrationFromProperties();

        Files.createDirectories(outputDir);

        Logger.reset();
        Logger.disableConsoleCapture();
        Logger.setReplay(true);
        Logger.setReplaySource(new RLOGReplay(logPath.toString()));
        Logger.addDataReceiver(new RLOGWriter(outputDir.toString().replace('\\', '/'), outputName));
        Logger.recordMetadata("ReplaySource", logPath.toAbsolutePath().toString());
        Logger.recordMetadata("ReplayGenerator", "LocalizationCandidateReplayCliMain");
        Logger.start();

        LocalizationCandidateCalculator calculator = new LocalizationCandidateCalculator();
        long cycles = 0;
        long acceptedCount = 0;

        try {
            while (Logger.isRunning()) {
                LogTable entry = Logger.getEntry();

                Pose2d pinpointPose = readPose2d(entry, "RealOutputs/Localization/Primary/Pose2d",
                        entry.get("RealOutputs/Localization/Primary/Valid", 0.0) > 0.5);
                Pose2d mt1Pose = readPose2d(entry, "RealOutputs/Localization/Limelight/MT1/Pose2d",
                        entry.get("RealOutputs/Localization/Limelight/MT1/Valid", 0.0) > 0.5);
                int tagCount = entry.get("RealOutputs/Localization/Candidates/Diagnostics/TagCount", 0);
                double planarDistanceIn = entry.get("RealOutputs/Localization/Candidates/Diagnostics/PlanarDistanceIn", Double.NaN);
                double robotSpeedInS = entry.get("RealOutputs/Localization/Candidates/Diagnostics/RobotSpeedInS",
                        entry.get("RealOutputs/Pinpoint/Speed", Double.NaN));

                LocalizationCandidateCalculator.CandidateResult result = calculator.update(
                        pinpointPose,
                        mt1Pose,
                        tagCount,
                        planarDistanceIn,
                        robotSpeedInS
                );
                calculator.recordOutputs("Localization/Candidates", result);
                if (result.accepted) {
                    acceptedCount++;
                }

                Logger.periodicAfterUser(0.0, 0.0);
                cycles++;
                if (!Logger.isRunning()) {
                    break;
                }
                Logger.periodicBeforeUser();
            }
        } finally {
            Logger.end();
        }

        System.out.println("[LocalizationReplay] input=" + logPath.toAbsolutePath());
        System.out.println("[LocalizationReplay] outputDir=" + outputDir.toAbsolutePath());
        System.out.println("[LocalizationReplay] outputName=" + outputName);
        System.out.println("[LocalizationReplay] mt1CalibrationXMeters=" + LocalizationCandidateCalculator.mt1FieldOffsetXMeters);
        System.out.println("[LocalizationReplay] mt1CalibrationYMeters=" + LocalizationCandidateCalculator.mt1FieldOffsetYMeters);
        System.out.println("[LocalizationReplay] mt1CalibrationHeadingDeg=" + LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg);
        System.out.println("[LocalizationReplay] cycles=" + cycles);
        System.out.println("[LocalizationReplay] accepted=" + acceptedCount);
    }

        private static void configureCalibrationFromProperties() {
        LocalizationCandidateCalculator.mt1FieldOffsetXMeters = resolveDouble(
            CAL_X_PROPERTY,
            LocalizationCandidateCalculator.mt1FieldOffsetXMeters
        );
        LocalizationCandidateCalculator.mt1FieldOffsetYMeters = resolveDouble(
            CAL_Y_PROPERTY,
            LocalizationCandidateCalculator.mt1FieldOffsetYMeters
        );
        LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg = resolveDouble(
            CAL_HEADING_PROPERTY,
            LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg
        );
        }

    private static Pose2d readPose2d(LogTable entry, String key, boolean valid) {
        if (!valid) {
            return null;
        }
        return entry.get(key, Pose2d.kZero);
    }

    private static Path resolveRequiredPath(String prop) {
        String raw = System.getProperty(prop);
        if (raw == null || raw.trim().isEmpty()) {
            throw new IllegalArgumentException("Missing required property: " + prop);
        }
        Path path = Paths.get(raw.trim());
        if (!Files.exists(path)) {
            throw new IllegalArgumentException("File does not exist: " + path.toAbsolutePath());
        }
        return path;
    }

    private static Path resolveOutputDir() {
        String raw = System.getProperty(OUTPUT_DIR_PROPERTY, "build/psikitReplayOut").trim();
        return Paths.get(raw);
    }

    private static String resolveOutputName(Path inputLog) {
        String raw = System.getProperty(OUTPUT_NAME_PROPERTY, "").trim();
        if (!raw.isEmpty()) {
            return raw;
        }
        String fileName = inputLog.getFileName().toString();
        if (fileName.endsWith(".rlog")) {
            fileName = fileName.substring(0, fileName.length() - 5);
        }
        return fileName + "_LocalizationReplay";
    }

    private static double resolveDouble(String propertyName, double defaultValue) {
        String raw = System.getProperty(propertyName);
        if (raw == null || raw.trim().isEmpty()) {
            return defaultValue;
        }
        return Double.parseDouble(raw.trim());
    }
}