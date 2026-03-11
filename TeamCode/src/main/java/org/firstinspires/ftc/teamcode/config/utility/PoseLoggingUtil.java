package org.firstinspires.ftc.teamcode.config.utility;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

public final class PoseLoggingUtil {

    private static final double INCHES_TO_METERS = 0.0254;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN * 0.5;
    private static final LocalizationCandidateCalculator localizationCandidateCalculator = new LocalizationCandidateCalculator();
    private static GlobalVariables.AllianceColor lastLoadedBiasAlliance = null;
    private static boolean lastLoadedBiasValid = false;
    private static String lastLoadedBiasStatus = "not_loaded";

    public static boolean enableLocalizationCandidateLogging = true;

    private PoseLoggingUtil() {
    }

    public static void logMainPoseDetails(Robot robot) {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null || robot.outtake.vision == null) {
            return;
        }

        double turretCmdDeg = robot.outtake.turret.getCurrentDegrees();
        double turretEncDeg = robot.outtake.turret.getEncoderDegrees();
        double turretMappedDeg = robot.outtake.turret.getMappedEncoderTurretDegrees();
        double turretMappedErrDeg = robot.outtake.turret.getMappedEncoderErrorDeg(turretCmdDeg);

        Logger.recordOutput("Turret/CmdDeg", turretCmdDeg);
        Logger.recordOutput("Turret/EncoderDeg", turretEncDeg);
        Logger.recordOutput("Turret/MappedEncoderDeg", turretMappedDeg);
        Logger.recordOutput("Turret/MappedErrorDeg", turretMappedErrDeg);
        Logger.recordOutput("Turret/EncoderVoltage", robot.outtake.turret.getEncoderVoltage());
        Logger.recordOutput("Turret/AtMinLimit", robot.outtake.turret.atMinLimit(0.0) ? 1.0 : 0.0);
        Logger.recordOutput("Turret/AtMaxLimit", robot.outtake.turret.atMaxLimit(0.0) ? 1.0 : 0.0);
        Logger.recordOutput("Turret/AimLockEnabled", robot.outtake.isAimLockEnabled() ? 1.0 : 0.0);
        Logger.recordOutput("Turret/AimTarget", robot.outtake.getAimTarget().ordinal());
        Logger.recordOutput("Turret/AimSource", robot.outtake.getActiveLockSource().ordinal());
        Logger.recordOutput("Turret/AimOffsetDeg", Outtake.turretAimCommandOffsetDeg);

        Pose followerPose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;
        Pose2d pinpointFtcPose = null;
        if (followerPose != null) {
            Logger.recordOutput("Pinpoint/X", followerPose.getX());
            Logger.recordOutput("Pinpoint/Y", followerPose.getY());
            Logger.recordOutput("Pinpoint/HeadingDeg", Math.toDegrees(followerPose.getHeading()));
            Logger.recordOutput("Pinpoint/TotalHeadingDeg", Math.toDegrees(FollowerManager.follower.getTotalHeading()));
            pinpointFtcPose = toPose2dFromPedroInchesAsFtcCenterRotated(followerPose);
            Logger.recordOutput("Localization/Pinpoint/Pose2d", pinpointFtcPose);
        } else {
            Logger.recordOutput("Pinpoint/X", Double.NaN);
            Logger.recordOutput("Pinpoint/Y", Double.NaN);
            Logger.recordOutput("Pinpoint/HeadingDeg", Double.NaN);
            Logger.recordOutput("Pinpoint/TotalHeadingDeg", Double.NaN);
        }

        if (FollowerManager.follower != null && FollowerManager.follower.getVelocity() != null) {
            Logger.recordOutput("Pinpoint/VelX", FollowerManager.follower.getVelocity().getXComponent());
            Logger.recordOutput("Pinpoint/VelY", FollowerManager.follower.getVelocity().getYComponent());
            Logger.recordOutput("Pinpoint/Speed", FollowerManager.follower.getVelocity().getMagnitude());
        } else {
            Logger.recordOutput("Pinpoint/VelX", Double.NaN);
            Logger.recordOutput("Pinpoint/VelY", Double.NaN);
            Logger.recordOutput("Pinpoint/Speed", Double.NaN);
        }

        Logger.recordOutput("Limelight/HasTarget", robot.outtake.vision.hasTarget() ? 1.0 : 0.0);
        Logger.recordOutput("Limelight/TagId", robot.outtake.vision.getCurrentTagId());
        Logger.recordOutput("Limelight/Tx", robot.outtake.vision.getTx());

        LLResult latest = robot.outtake.vision.latest;
        Pose3D mt2Pose = getMt2Pose(latest);
        Pose3D mt1Pose = getMt1Pose(latest);
        Pose2d mt2PoseRaw = (mt2Pose != null) ? toPose2dFromLimelightMeters(mt2Pose) : null;
        Pose2d mt2PoseComp = (mt2Pose != null) ? toPose2dFromLimelightMetersCompensated(robot, mt2Pose) : null;
        Pose2d mt1PoseRaw = (mt1Pose != null) ? toPose2dFromLimelightMeters(mt1Pose) : null;
        Pose2d mt1PoseComp = (mt1Pose != null) ? toPose2dFromLimelightMetersCompensated(robot, mt1Pose) : null;

        Logger.recordOutput("Localization/Primary/Valid", pinpointFtcPose != null ? 1.0 : 0.0);
        if (pinpointFtcPose != null) {
            Logger.recordOutput("Localization/Primary/Pose2d", pinpointFtcPose);
            Logger.recordOutput("Localization/Primary/Source", "PinpointFTC");
        }

        Logger.recordOutput("Localization/Limelight/MT2/Valid", mt2Pose != null ? 1.0 : 0.0);
        if (mt2PoseRaw != null) {
            Logger.recordOutput("Localization/Limelight/MT2/Pose2dRaw", mt2PoseRaw);
        }
        if (mt2PoseComp != null) {
            Logger.recordOutput("Localization/Limelight/MT2/Pose2d", mt2PoseComp);
        }

        Logger.recordOutput("Localization/Limelight/MT1/Valid", mt1Pose != null ? 1.0 : 0.0);
        if (mt1PoseRaw != null) {
            Logger.recordOutput("Localization/Limelight/MT1/Pose2dRaw", mt1PoseRaw);
        }
        if (mt1PoseComp != null) {
            Logger.recordOutput("Localization/Limelight/MT1/Pose2d", mt1PoseComp);
        }

        Logger.recordOutput("Localization/LimelightMT2/Valid", mt2Pose != null ? 1.0 : 0.0);
        if (mt2PoseRaw != null) {
            Logger.recordOutput("Localization/LimelightMT2/Pose2dRaw", mt2PoseRaw);
        }
        if (mt2PoseComp != null) {
            Logger.recordOutput("Localization/LimelightMT2/Pose2d", mt2PoseComp);
        }

        Logger.recordOutput("Localization/LimelightMT1/Valid", mt1Pose != null ? 1.0 : 0.0);
        if (mt1PoseRaw != null) {
            Logger.recordOutput("Localization/LimelightMT1/Pose2dRaw", mt1PoseRaw);
        }
        if (mt1PoseComp != null) {
            Logger.recordOutput("Localization/LimelightMT1/Pose2d", mt1PoseComp);
        }

        if (enableLocalizationCandidateLogging) {
            logLocalizationCandidates(robot, latest, pinpointFtcPose, mt1PoseComp);
        }
    }

    private static void logLocalizationCandidates(Robot robot, LLResult latest, Pose2d pinpointFtcPose, Pose2d mt1PoseComp) {
        ensureAllianceBiasLoaded();

        int tagCount = getTagCount(latest);
        double planarDistanceIn = (robot != null && robot.outtake != null && robot.outtake.vision != null)
                ? robot.outtake.vision.getPlanarDistanceInches()
                : Double.NaN;
        double robotSpeedInS = getFollowerSpeedInS();
        LocalizationCandidateCalculator.CandidateResult result = localizationCandidateCalculator.update(
            pinpointFtcPose,
            mt1PoseComp,
            tagCount,
            planarDistanceIn,
            robotSpeedInS
        );
        localizationCandidateCalculator.recordOutputs("Localization/Candidates", result);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Calibration/Alliance", GlobalVariables.getAllianceColorName());
        Logger.recordOutput("Localization/Candidates/Diagnostics/Calibration/LoadedFromFile", lastLoadedBiasValid ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Calibration/LoadStatus", lastLoadedBiasStatus);
    }

    private static void ensureAllianceBiasLoaded() {
        GlobalVariables.AllianceColor allianceColor = GlobalVariables.getAllianceColor();
        if (allianceColor == lastLoadedBiasAlliance) {
            return;
        }

        lastLoadedBiasAlliance = allianceColor;
        try {
            LocalizationBiasFileStore.Bias bias = LocalizationBiasFileStore.loadAllianceBias(allianceColor);
            if (bias.valid) {
                LocalizationBiasFileStore.applyBiasToCalculator(bias);
                lastLoadedBiasValid = true;
                lastLoadedBiasStatus = "loaded";
            } else {
                LocalizationCandidateCalculator.mt1FieldOffsetXMeters = 0.0;
                LocalizationCandidateCalculator.mt1FieldOffsetYMeters = 0.0;
                LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg = 0.0;
                lastLoadedBiasValid = false;
                lastLoadedBiasStatus = "missing";
            }
        } catch (Exception e) {
            LocalizationCandidateCalculator.mt1FieldOffsetXMeters = 0.0;
            LocalizationCandidateCalculator.mt1FieldOffsetYMeters = 0.0;
            LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg = 0.0;
            lastLoadedBiasValid = false;
            lastLoadedBiasStatus = "error:" + e.getClass().getSimpleName();
        }
    }

    private static Pose3D getMt2Pose(LLResult latest) {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose_MT2();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private static Pose3D getMt1Pose(LLResult latest) {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private static Pose2d toPose2dFromPedroInchesAsFtcCenterRotated(Pose pedroPose) {
        double pedroXIn = pedroPose.getX();
        double pedroYIn = pedroPose.getY();
        double pedroHeadingRad = pedroPose.getHeading();

        double ftcXIn = FIELD_HALF_IN - pedroYIn;
        double ftcYIn = pedroXIn - FIELD_HALF_IN;
        double ftcHeadingRad = wrapRad(pedroHeadingRad + (Math.PI * 0.5));

        return new Pose2d(
                ftcXIn * INCHES_TO_METERS,
                ftcYIn * INCHES_TO_METERS,
                Rotation2d.fromRadians(ftcHeadingRad)
        );
    }

    private static Pose2d toPose2dFromLimelightMeters(Pose3D llPose) {
        double xMeters = llPose.getPosition().x;
        double yMeters = llPose.getPosition().y;
        double headingRad = Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES));
        return new Pose2d(xMeters, yMeters, Rotation2d.fromRadians(headingRad));
    }

    private static Pose2d toPose2dFromLimelightMetersCompensated(Robot robot, Pose3D llPose) {
        if (robot == null || robot.outtake == null || robot.outtake.vision == null || llPose == null) {
            return toPose2dFromLimelightMeters(llPose);
        }

        double[] corrected = robot.outtake.vision.getTurretCompensatedPose2dMetersFromPose3d(llPose);
        if (corrected == null || corrected.length < 3) {
            return toPose2dFromLimelightMeters(llPose);
        }

        return new Pose2d(
                corrected[0],
                corrected[1],
                Rotation2d.fromRadians(Math.toRadians(corrected[2]))
        );
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static int getTagCount(LLResult latest) {
        if (latest == null || !latest.isValid() || latest.getFiducialResults() == null) {
            return 0;
        }
        return latest.getFiducialResults().size();
    }

    private static double getFollowerSpeedInS() {
        if (FollowerManager.follower == null || FollowerManager.follower.getVelocity() == null) {
            return Double.NaN;
        }
        return FollowerManager.follower.getVelocity().getMagnitude();
    }
}
