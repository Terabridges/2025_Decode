package org.firstinspires.ftc.teamcode.config.utility;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Vision;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

@Configurable
public final class LocalizationDriftCorrection {

    public static boolean enabled = false;
    public static int minTagCount = 1;
    public static int preferredMultiTagCount = 2;
    public static double maxPlanarDistanceIn = 120.0;
    public static double maxRobotSpeedInS = 18.0;
    public static double maxVisionTranslationErrorM = 0.75;
    public static double translationAlphaSingleTag = 0.04;
    public static double translationAlphaMultiTag = 0.10;
    public static int requiredConsecutiveAcceptedFrames = 3;
    public static double maxAppliedStepMeters = 0.030;
    public static boolean requireTurretNearForward = false;
    public static double turretNearForwardToleranceDeg = 12.0;

    private static final double INCHES_TO_METERS = 0.0254;
    private static final double METERS_TO_INCHES = 39.3701;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN * 0.5;

    private static int consecutiveAcceptedFrames = 0;

    private LocalizationDriftCorrection() {
    }

    public static void apply(Vision vision, Turret turret) {
        DriftCorrectionResult result = new DriftCorrectionResult();
        result.enabled = enabled;

        LocalizationBiasFileStore.LoadResult loadResult = LocalizationBiasFileStore.ensureAllianceBiasLoaded(GlobalVariables.getAllianceColor());
        result.biasLoadedFromFile = loadResult.loadedFromFile;
        result.biasLoadStatus = loadResult.status;

        if (!enabled || vision == null || follower == null) {
            consecutiveAcceptedFrames = 0;
            result.rejectDisabled = !enabled;
            result.rejectFollowerMissing = (follower == null);
            recordOutputs(result);
            return;
        }

        Pose followerPose = follower.getPose();
        if (followerPose == null) {
            consecutiveAcceptedFrames = 0;
            result.rejectFollowerMissing = true;
            recordOutputs(result);
            return;
        }

        Pose2d followerFtcPose = toPose2dFromPedroInchesAsFtcCenterRotated(followerPose);
        result.followerPose = followerFtcPose;

        Pose3D mt1Pose3d = vision.getLatestMt1Pose();
        if (mt1Pose3d == null) {
            consecutiveAcceptedFrames = 0;
            result.rejectNoObservation = true;
            recordOutputs(result);
            return;
        }

        Pose2d mt1Pose = toPose2dFromLimelightMetersCompensated(vision, mt1Pose3d);
        Pose2d mt1CalibratedPose = LocalizationCandidateCalculator.applyMt1Calibration(mt1Pose);
        result.mt1Pose = mt1CalibratedPose;
        if (mt1CalibratedPose == null) {
            consecutiveAcceptedFrames = 0;
            result.rejectNoObservation = true;
            recordOutputs(result);
            return;
        }

        result.tagCount = getTagCount(vision);
        result.planarDistanceIn = vision.getPlanarDistanceInches();
        result.robotSpeedInS = getFollowerSpeedInS();
        result.translationErrorM = distanceMeters(followerFtcPose, mt1CalibratedPose);

        result.rejectTagCount = result.tagCount < minTagCount;
        result.rejectDistance = Double.isFinite(result.planarDistanceIn) && result.planarDistanceIn > maxPlanarDistanceIn;
        result.rejectRobotSpeed = Double.isFinite(result.robotSpeedInS) && result.robotSpeedInS > maxRobotSpeedInS;
        result.rejectTranslationJump = result.translationErrorM > maxVisionTranslationErrorM;
        result.rejectTurretOrientation = requireTurretNearForward && turret != null
                && Math.abs(wrapSignedDegrees(turret.getCurrentDegrees() - Turret.turretForwardDeg)) > turretNearForwardToleranceDeg;

        result.accepted = !result.rejectTagCount
                && !result.rejectDistance
                && !result.rejectRobotSpeed
                && !result.rejectTranslationJump
                && !result.rejectTurretOrientation;

        if (!result.accepted) {
            consecutiveAcceptedFrames = 0;
            result.consecutiveAcceptedFrames = 0;
            recordOutputs(result);
            return;
        }

        consecutiveAcceptedFrames++;
        result.consecutiveAcceptedFrames = consecutiveAcceptedFrames;
        result.stableAccepted = consecutiveAcceptedFrames >= Math.max(1, requiredConsecutiveAcceptedFrames);
        result.translationAlpha = (result.tagCount >= preferredMultiTagCount)
                ? translationAlphaMultiTag
                : translationAlphaSingleTag;

        if (!result.stableAccepted) {
            recordOutputs(result);
            return;
        }

        double desiredDx = mt1CalibratedPose.getX() - followerFtcPose.getX();
        double desiredDy = mt1CalibratedPose.getY() - followerFtcPose.getY();
        double rawStepX = desiredDx * clamp01(result.translationAlpha);
        double rawStepY = desiredDy * clamp01(result.translationAlpha);
        double rawStepMag = Math.hypot(rawStepX, rawStepY);
        double stepScale = 1.0;
        if (rawStepMag > maxAppliedStepMeters && rawStepMag > 1e-9) {
            stepScale = maxAppliedStepMeters / rawStepMag;
        }

        double appliedStepX = rawStepX * stepScale;
        double appliedStepY = rawStepY * stepScale;
        result.appliedStepMeters = Math.hypot(appliedStepX, appliedStepY);
        result.correctedPose = new Pose2d(
                followerFtcPose.getX() + appliedStepX,
                followerFtcPose.getY() + appliedStepY,
                Rotation2d.fromRadians(followerFtcPose.getRotation().getRadians())
        );

        follower.setPose(toPedroPoseFromFtcCenteredMeters(result.correctedPose));
        result.applied = true;
        recordOutputs(result);
    }

    private static void recordOutputs(DriftCorrectionResult result) {
        String base = "Localization/DriftTrim";
        Logger.recordOutput(base + "/Enabled", result.enabled ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Accepted", result.accepted ? 1.0 : 0.0);
        Logger.recordOutput(base + "/StableAccepted", result.stableAccepted ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Applied", result.applied ? 1.0 : 0.0);
        Logger.recordOutput(base + "/ConsecutiveAcceptedFrames", result.consecutiveAcceptedFrames);
        Logger.recordOutput(base + "/Diagnostics/TagCount", result.tagCount);
        Logger.recordOutput(base + "/Diagnostics/PlanarDistanceIn", result.planarDistanceIn);
        Logger.recordOutput(base + "/Diagnostics/RobotSpeedInS", result.robotSpeedInS);
        Logger.recordOutput(base + "/Diagnostics/TranslationErrorM", result.translationErrorM);
        Logger.recordOutput(base + "/Diagnostics/TranslationAlpha", result.translationAlpha);
        Logger.recordOutput(base + "/Diagnostics/AppliedStepMeters", result.appliedStepMeters);
        Logger.recordOutput(base + "/Diagnostics/Calibration/LoadedFromFile", result.biasLoadedFromFile ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Calibration/LoadStatus", result.biasLoadStatus);
        Logger.recordOutput(base + "/Diagnostics/Reject/Disabled", result.rejectDisabled ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/FollowerMissing", result.rejectFollowerMissing ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/NoObservation", result.rejectNoObservation ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/TagCount", result.rejectTagCount ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/Distance", result.rejectDistance ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/RobotSpeed", result.rejectRobotSpeed ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/TranslationJump", result.rejectTranslationJump ? 1.0 : 0.0);
        Logger.recordOutput(base + "/Diagnostics/Reject/TurretOrientation", result.rejectTurretOrientation ? 1.0 : 0.0);

        if (result.followerPose != null) {
            Logger.recordOutput(base + "/FollowerBefore/Pose2d", result.followerPose);
        }
        if (result.mt1Pose != null) {
            Logger.recordOutput(base + "/VisionCalibrated/Pose2d", result.mt1Pose);
        }
        if (result.correctedPose != null) {
            Logger.recordOutput(base + "/FollowerAfter/Pose2d", result.correctedPose);
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

    private static Pose toPedroPoseFromFtcCenteredMeters(Pose2d ftcPose) {
        double ftcXIn = ftcPose.getX() * METERS_TO_INCHES;
        double ftcYIn = ftcPose.getY() * METERS_TO_INCHES;
        double ftcHeadingRad = ftcPose.getRotation().getRadians();

        double pedroXIn = ftcYIn + FIELD_HALF_IN;
        double pedroYIn = FIELD_HALF_IN - ftcXIn;
        double pedroHeadingRad = wrapRad(ftcHeadingRad - (Math.PI * 0.5));

        return new Pose(pedroXIn, pedroYIn, pedroHeadingRad);
    }

    private static Pose2d toPose2dFromLimelightMetersCompensated(Vision vision, Pose3D llPose) {
        double[] corrected = vision.getTurretCompensatedPose2dMetersFromPose3d(llPose);
        if (corrected == null || corrected.length < 3) {
            return new Pose2d(
                    llPose.getPosition().x,
                    llPose.getPosition().y,
                    Rotation2d.fromRadians(Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES)))
            );
        }

        return new Pose2d(
                corrected[0],
                corrected[1],
                Rotation2d.fromRadians(Math.toRadians(corrected[2]))
        );
    }

    private static int getTagCount(Vision vision) {
        if (vision.latest == null || !vision.latest.isValid() || vision.latest.getFiducialResults() == null) {
            return 0;
        }
        return vision.latest.getFiducialResults().size();
    }

    private static double getFollowerSpeedInS() {
        if (follower == null || follower.getVelocity() == null) {
            return Double.NaN;
        }
        return follower.getVelocity().getMagnitude();
    }

    private static double distanceMeters(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static final class DriftCorrectionResult {
        boolean enabled;
        boolean accepted;
        boolean stableAccepted;
        boolean applied;
        boolean biasLoadedFromFile;
        String biasLoadStatus = "not_loaded";
        boolean rejectDisabled;
        boolean rejectFollowerMissing;
        boolean rejectNoObservation;
        boolean rejectTagCount;
        boolean rejectDistance;
        boolean rejectRobotSpeed;
        boolean rejectTranslationJump;
        boolean rejectTurretOrientation;
        int consecutiveAcceptedFrames;
        int tagCount;
        double planarDistanceIn = Double.NaN;
        double robotSpeedInS = Double.NaN;
        double translationErrorM = Double.NaN;
        double translationAlpha = Double.NaN;
        double appliedStepMeters = Double.NaN;
        Pose2d followerPose;
        Pose2d mt1Pose;
        Pose2d correctedPose;
    }
}
