package org.firstinspires.ftc.teamcode.logging;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.psilynx.psikit.core.Logger;

/** Logs the Pedro {@link Follower} pose using AdvantageScope Pose2d schema. */
public final class PsiKitPedroFollowerPoseLogger {

    private final AdvantageScopePose2dInputs pose2d = new AdvantageScopePose2dInputs();
    private final String posePath;

    public PsiKitPedroFollowerPoseLogger(String posePath) {
        this.posePath = posePath;
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
        Logger.processInputs(posePath, pose2d);
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}
