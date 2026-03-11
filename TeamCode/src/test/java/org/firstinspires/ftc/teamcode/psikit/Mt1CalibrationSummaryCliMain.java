package org.firstinspires.ftc.teamcode.psikit;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public final class Mt1CalibrationSummaryCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String MAX_SPEED_PROPERTY = "mt1CalibrationMaxRobotSpeedInS";
    private static final String REQUIRE_ACCEPTED_PROPERTY = "mt1CalibrationRequireAccepted";

    private Mt1CalibrationSummaryCliMain() {
    }

    public static void main(String[] args) {
        Path logPath = resolveRequiredPath(LOG_PATH_PROPERTY);
        double maxRobotSpeedInS = resolveDouble(MAX_SPEED_PROPERTY, 3.0);
        boolean requireAccepted = resolveBoolean(REQUIRE_ACCEPTED_PROPERTY, true);

        long entries = 0;
        long samples = 0;
        double sumDx = 0.0;
        double sumDy = 0.0;
        double sumDist = 0.0;
        double sumHeadingDeg = 0.0;

        RLOGReplay replay = new RLOGReplay(logPath.toString());
        replay.start();
        try {
            while (true) {
                LogTable entry = replay.getEntry();
                if (entry == null) {
                    break;
                }
                entries++;

                boolean primaryValid = entry.get("RealOutputs/Localization/Primary/Valid", 0.0) > 0.5;
                boolean mt1Valid = entry.get("RealOutputs/Localization/Limelight/MT1/Valid", 0.0) > 0.5;
                if (!primaryValid || !mt1Valid) {
                    continue;
                }

                if (requireAccepted && entry.get("RealOutputs/Localization/Candidates/MT1Smoothed/Accepted", 0.0) <= 0.5) {
                    continue;
                }

                double robotSpeedInS = entry.get(
                        "RealOutputs/Localization/Candidates/Diagnostics/RobotSpeedInS",
                        Double.NaN
                );
                if (Double.isFinite(robotSpeedInS) && robotSpeedInS > maxRobotSpeedInS) {
                    continue;
                }

                Pose2d primaryPose = entry.get("RealOutputs/Localization/Primary/Pose2d", Pose2d.kZero);
                Pose2d mt1Pose = entry.get("RealOutputs/Localization/Limelight/MT1/Pose2d", Pose2d.kZero);

                double dx = primaryPose.getX() - mt1Pose.getX();
                double dy = primaryPose.getY() - mt1Pose.getY();
                double headingDeg = Math.toDegrees(wrapRad(
                        primaryPose.getRotation().getRadians() - mt1Pose.getRotation().getRadians()
                ));

                samples++;
                sumDx += dx;
                sumDy += dy;
                sumDist += Math.hypot(dx, dy);
                sumHeadingDeg += headingDeg;
            }
        } finally {
            replay.end();
        }

        System.out.println("[Mt1CalibrationSummary] log=" + logPath.toAbsolutePath());
        System.out.println("[Mt1CalibrationSummary] entries=" + entries);
        System.out.println("[Mt1CalibrationSummary] samples=" + samples);
        System.out.println("[Mt1CalibrationSummary] maxRobotSpeedInS=" + maxRobotSpeedInS);
        System.out.println("[Mt1CalibrationSummary] requireAccepted=" + requireAccepted);
        if (samples <= 0) {
            System.out.println("[Mt1CalibrationSummary] No matching samples.");
            return;
        }

        double avgDx = sumDx / samples;
        double avgDy = sumDy / samples;
        double avgDist = sumDist / samples;
        double avgHeadingDeg = sumHeadingDeg / samples;

        System.out.println("[Mt1CalibrationSummary] suggestedOffsetXMeters=" + avgDx);
        System.out.println("[Mt1CalibrationSummary] suggestedOffsetYMeters=" + avgDy);
        System.out.println("[Mt1CalibrationSummary] suggestedOffsetHeadingDeg=" + avgHeadingDeg);
        System.out.println("[Mt1CalibrationSummary] avgDistanceMeters=" + avgDist);
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
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

    private static double resolveDouble(String propertyName, double defaultValue) {
        String raw = System.getProperty(propertyName);
        if (raw == null || raw.trim().isEmpty()) {
            return defaultValue;
        }
        return Double.parseDouble(raw.trim());
    }

    private static boolean resolveBoolean(String propertyName, boolean defaultValue) {
        String raw = System.getProperty(propertyName);
        if (raw == null || raw.trim().isEmpty()) {
            return defaultValue;
        }
        String trimmed = raw.trim().toLowerCase();
        return "1".equals(trimmed) || "true".equals(trimmed) || "yes".equals(trimmed) || "on".equals(trimmed);
    }
}