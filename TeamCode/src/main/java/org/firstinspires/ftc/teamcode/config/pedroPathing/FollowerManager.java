package org.firstinspires.ftc.teamcode.config.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.utility.Drawing;

/**
 * Centralized owner for the Pedro follower so auto and teleop can share the same instance.
 */
public class FollowerManager {

    public static Follower follower;
    public static PoseHistory poseHistory;
    public static TelemetryManager telemetryM;

    private FollowerManager() {}

    /**
     * Always (re)build the follower using the provided hardware map, replacing any existing one.
     *
     * @param hardwareMap active opmode hardware map
     * @param startPose   optional starting pose to seed (nullable)
     * @return live follower instance
     */
    public static synchronized Follower initFollower(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.createFollower(hardwareMap);
        poseHistory = follower.getPoseHistory();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        if (startPose != null) {
            follower.setStartingPose(startPose);
        }

        return follower;
    }

    /**
     * Fetch the existing follower if present; otherwise create one seeded with the provided pose.
     *
     * @param hardwareMap       active opmode hardware map
     * @param defaultStartPose  pose to use when creating a fresh follower (nullable)
     * @return live follower instance
     */
    public static synchronized Follower getFollower(HardwareMap hardwareMap, Pose defaultStartPose) {
        if (follower == null) {
            return initFollower(hardwareMap, defaultStartPose);
        }
        return follower;
    }

    /**
     * Convenience overload for callers that do not need to seed a starting pose.
     */
    public static Follower getFollower(HardwareMap hardwareMap) {
        return getFollower(hardwareMap, null);
    }

    public static void setStartPose(Pose pose) {
        if (follower != null && pose != null) {
            follower.setStartingPose(pose);
        }
    }

    public static void drawCurrent() {
        if (follower == null) {
            return;
        }
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
    }

    public static void drawCurrentAndHistory() {
        if (poseHistory != null) {
            Drawing.drawPoseHistory(poseHistory);
        }
        drawCurrent();
    }
}
