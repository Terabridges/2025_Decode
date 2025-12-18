package org.firstinspires.ftc.teamcode.logging;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.psilynx.psikit.core.Logger;

/** Logs the Pedro {@link Follower} pose using AdvantageScope Pose2d schema. */
public final class PsiKitPedroFollowerPoseLogger {

    private final StructPoseInputs pose2d;
    private final String tableKey;

    public PsiKitPedroFollowerPoseLogger(String posePath) {
        String normalized = normalizeKey(posePath);
        int lastSlash = normalized.lastIndexOf('/');
        if (lastSlash > 0) {
            this.tableKey = normalized.substring(0, lastSlash);
            String fieldKey = normalized.substring(lastSlash + 1);
            this.pose2d = new StructPoseInputs(fieldKey, null);
        } else {
            // Fallback: keep everything under the provided key.
            this.tableKey = normalized;
            this.pose2d = new StructPoseInputs("value", null);
        }
    }

    public void log(Follower follower) {
        if (follower == null) {
            return;
        }

        Pose pose = follower.getPose();
        if (pose == null) {
            return;
        }

        // Pedro uses inches/radians by default.
        double xMeters = inchesToMeters(pose.getX());
        double yMeters = inchesToMeters(pose.getY());
        double headingRad = pose.getHeading();

        pose2d.set(xMeters, yMeters, headingRad);
        Logger.processInputs(tableKey, pose2d);
    }

    private static String normalizeKey(String key) {
        if (key == null) {
            return "";
        }
        String result = key.trim();
        while (result.endsWith("/")) {
            result = result.substring(0, result.length() - 1);
        }
        return result;
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}
