package org.firstinspires.ftc.teamcode.logging;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

/**
 * Logs Pose2d/Pose3d as a single struct value each (so AdvantageScope can select the whole pose).
 *
 * <p>Also removes legacy component fields that were previously logged as nested numbers.
 */
public final class StructPoseInputs implements LoggableInputs {

    private final String pose2dKey;
    private final String pose3dKey;

    private double xMeters;
    private double yMeters;
    private double headingRad;

    public StructPoseInputs(String pose2dKey, String pose3dKey) {
        this.pose2dKey = pose2dKey;
        this.pose3dKey = pose3dKey;
    }

    public void set(double xMeters, double yMeters, double headingRad) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.headingRad = headingRad;
    }

    @Override
    public void toLog(LogTable table) {
        if (pose2dKey != null && !pose2dKey.isEmpty()) {
            Pose2d pose2d = new Pose2d(xMeters, yMeters, new Rotation2d(headingRad));
            table.put(pose2dKey, pose2d);
            removeLegacyPose2d(table, pose2dKey);
        }

        if (pose3dKey != null && !pose3dKey.isEmpty()) {
            Pose2d pose2dFor3d = new Pose2d(xMeters, yMeters, new Rotation2d(headingRad));
            Pose3d pose3d = new Pose3d(pose2dFor3d);
            table.put(pose3dKey, pose3d);
            removeLegacyPose3d(table, pose3dKey);
        }
    }

    @Override
    public void fromLog(LogTable table) {
        // Not used in this FTC project (no replay).
    }

    private static void removeLegacyPose2d(LogTable table, String key) {
        table.remove(key + "/translation/x");
        table.remove(key + "/translation/y");
        table.remove(key + "/rotation/value");

        // Older incorrect key names.
        table.remove(key + "/x");
        table.remove(key + "/y");
        table.remove(key + "/theta");
    }

    private static void removeLegacyPose3d(LogTable table, String key) {
        table.remove(key + "/translation/x");
        table.remove(key + "/translation/y");
        table.remove(key + "/translation/z");

        // Current nested quaternion layout we used previously.
        table.remove(key + "/rotation/q/w");
        table.remove(key + "/rotation/q/x");
        table.remove(key + "/rotation/q/y");
        table.remove(key + "/rotation/q/z");

        // Older incorrect quaternion key names.
        table.remove(key + "/rotation/qw");
        table.remove(key + "/rotation/qx");
        table.remove(key + "/rotation/qy");
        table.remove(key + "/rotation/qz");
    }
}
