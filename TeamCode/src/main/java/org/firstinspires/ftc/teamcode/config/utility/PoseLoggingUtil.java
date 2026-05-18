package org.firstinspires.ftc.teamcode.config.utility;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;

public final class PoseLoggingUtil {

    private static final double INCHES_TO_METERS = 0.0254;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN * 0.5;
    private static final CandidatePoseState mt1SmoothedState = new CandidatePoseState();

    public static boolean enableLocalizationCandidateLogging = true;
    public static int mt1MinTagCount = 1;
    public static int mt1PreferredMultiTagCount = 2;
    public static double mt1MaxPlanarDistanceIn = 140.0;
    public static double mt1MaxRobotSpeedInS = 24.0;
    public static double mt1MaxTranslationJumpMeters = 1.25;
    public static double mt1MaxHeadingJumpDeg = 75.0;
    public static double mt1StateResetJumpMeters = 2.0;
    public static double mt1StateResetHeadingJumpDeg = 120.0;
    public static double mt1PositionAlphaSingleTag = 0.08;
    public static double mt1PositionAlphaMultiTag = 0.22;
    public static double mt1HeadingAlphaSingleTag = 0.04;
    public static double mt1HeadingAlphaMultiTag = 0.14;
    public static double hybridTranslationGain = 0.45;
    public static double hybridVisionHeadingGainSingleTag = 0.08;
    public static double hybridVisionHeadingGainMultiTag = 0.22;
    public static double vizChassisHeightMeters = 0.0;
    public static double vizTurretHeightMeters = 0.22;
    public static double vizTurretPivotOffsetXMeters = 0.0;
    public static double vizTurretPivotOffsetYMeters = 0.0;
    public static double vizTargetHeightMeters = 0.18;
    public static double vizDefaultTargetRangeMeters = 3.0;
    public static boolean vizLogDualTargetSigns = true;

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
        Logger.recordOutput("Turret/AimOffsetDeg", Outtake.getTotalTurretAimCommandOffsetDeg());
        Logger.recordOutput("Turret/AimOffsetDeg/VisionB", Outtake.turretAimCommandOffsetDeg);
        Logger.recordOutput("Turret/AimOffsetDeg/AutoVisionBias", Outtake.turretAimAutoVisionBiasDeg);
        Logger.recordOutput("Turret/AimOffsetDeg/Trim", Outtake.turretAimTrimOffsetDeg);

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

        logVizPose3d(robot, pinpointFtcPose, turretMappedDeg);

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
        int tagCount = getTagCount(latest);
        double planarDistanceIn = (robot != null && robot.outtake != null && robot.outtake.vision != null)
                ? robot.outtake.vision.getPlanarDistanceInches()
                : Double.NaN;
        double robotSpeedInS = getFollowerSpeedInS();

        boolean rejectNoObservation = (mt1PoseComp == null);
        boolean rejectTagCount = !rejectNoObservation && tagCount < mt1MinTagCount;
        boolean rejectDistance = !rejectNoObservation && Double.isFinite(planarDistanceIn) && planarDistanceIn > mt1MaxPlanarDistanceIn;
        boolean rejectRobotSpeed = !rejectNoObservation && Double.isFinite(robotSpeedInS) && robotSpeedInS > mt1MaxRobotSpeedInS;

        double translationErrorMeters = Double.NaN;
        double headingErrorDeg = Double.NaN;
        boolean rejectTranslationJump = false;
        boolean rejectHeadingJump = false;
        if (pinpointFtcPose != null && mt1PoseComp != null) {
            translationErrorMeters = distanceMeters(pinpointFtcPose, mt1PoseComp);
            headingErrorDeg = Math.toDegrees(angleDifferenceRad(
                    pinpointFtcPose.getRotation().getRadians(),
                    mt1PoseComp.getRotation().getRadians()
            ));
            rejectTranslationJump = translationErrorMeters > mt1MaxTranslationJumpMeters;
            rejectHeadingJump = Math.abs(headingErrorDeg) > mt1MaxHeadingJumpDeg;
        }

        boolean accepted = !rejectNoObservation
                && !rejectTagCount
                && !rejectDistance
                && !rejectRobotSpeed
                && !rejectTranslationJump
                && !rejectHeadingJump;

        Logger.recordOutput("Localization/Candidates/MT1Smoothed/Accepted", accepted ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/MT1Smoothed/Valid", mt1SmoothedState.valid ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/TagCount", tagCount);
        Logger.recordOutput("Localization/Candidates/Diagnostics/PlanarDistanceIn", planarDistanceIn);
        Logger.recordOutput("Localization/Candidates/Diagnostics/RobotSpeedInS", robotSpeedInS);
        Logger.recordOutput("Localization/Candidates/Diagnostics/PinpointVsMT1TranslationErrorM", translationErrorMeters);
        Logger.recordOutput("Localization/Candidates/Diagnostics/PinpointVsMT1HeadingErrorDeg", headingErrorDeg);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Reject/NoObservation", rejectNoObservation ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Reject/TagCount", rejectTagCount ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Reject/Distance", rejectDistance ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Reject/RobotSpeed", rejectRobotSpeed ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Reject/TranslationJump", rejectTranslationJump ? 1.0 : 0.0);
        Logger.recordOutput("Localization/Candidates/Diagnostics/Reject/HeadingJump", rejectHeadingJump ? 1.0 : 0.0);

        if (accepted && mt1PoseComp != null) {
            double positionAlpha = getPositionAlpha(tagCount);
            double headingAlpha = getHeadingAlpha(tagCount);
            Logger.recordOutput("Localization/Candidates/Diagnostics/PositionAlpha", positionAlpha);
            Logger.recordOutput("Localization/Candidates/Diagnostics/HeadingAlpha", headingAlpha);
            updateMt1SmoothedState(mt1PoseComp, positionAlpha, headingAlpha);
        }

        if (mt1SmoothedState.valid) {
            Pose2d mt1SmoothedPose = mt1SmoothedState.toPose2d();
            Logger.recordOutput("Localization/Candidates/MT1Smoothed/Pose2d", mt1SmoothedPose);

            if (pinpointFtcPose != null) {
                Pose2d hybridPinpointHeading = new Pose2d(
                        lerp(pinpointFtcPose.getX(), mt1SmoothedPose.getX(), hybridTranslationGain),
                        lerp(pinpointFtcPose.getY(), mt1SmoothedPose.getY(), hybridTranslationGain),
                        Rotation2d.fromRadians(pinpointFtcPose.getRotation().getRadians())
                );
                Logger.recordOutput("Localization/Candidates/HybridPinpointHeading/Pose2d", hybridPinpointHeading);

                double visionHeadingGain = getHybridVisionHeadingGain(tagCount);
                Pose2d hybridVisionHeading = new Pose2d(
                        hybridPinpointHeading.getX(),
                        hybridPinpointHeading.getY(),
                        Rotation2d.fromRadians(angleLerp(
                                pinpointFtcPose.getRotation().getRadians(),
                                mt1SmoothedPose.getRotation().getRadians(),
                                visionHeadingGain
                        ))
                );
                Logger.recordOutput("Localization/Candidates/HybridVisionHeading/Pose2d", hybridVisionHeading);
                Logger.recordOutput("Localization/Candidates/Diagnostics/HybridVisionHeadingGain", visionHeadingGain);
                Logger.recordOutput("Localization/Candidates/Diagnostics/PinpointVsSmoothedTranslationErrorM",
                        distanceMeters(pinpointFtcPose, mt1SmoothedPose));
                Logger.recordOutput("Localization/Candidates/Diagnostics/PinpointVsSmoothedHeadingErrorDeg",
                        Math.toDegrees(angleDifferenceRad(
                                pinpointFtcPose.getRotation().getRadians(),
                                mt1SmoothedPose.getRotation().getRadians()
                        )));
            }
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

    private static void logVizPose3d(Robot robot, Pose2d chassisPose2d, double turretMappedDeg) {
        Logger.recordOutput("Viz/Robot/ChassisPose3d/Valid", chassisPose2d != null ? 1.0 : 0.0);
        if (chassisPose2d == null) {
            Logger.recordOutput("Viz/Robot/TurretPose3d/Valid", 0.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3d/Valid", 0.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dPlusTx/Valid", 0.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dMinusTx/Valid", 0.0);
            return;
        }

        double chassisYawRad = chassisPose2d.getRotation().getRadians();
        Pose3d chassisPose3d = new Pose3d(
                chassisPose2d.getX(),
                chassisPose2d.getY(),
                vizChassisHeightMeters,
                new Rotation3d(0.0, 0.0, chassisYawRad)
        );
        Logger.recordOutput("Viz/Robot/ChassisPose3d", chassisPose3d);

        double turretRelYawRad = Math.toRadians(wrapSignedDegrees(turretMappedDeg - Turret.turretForwardDeg));
        double turretWorldYawRad = wrapRad(chassisYawRad + turretRelYawRad);

        double cosChassis = Math.cos(chassisYawRad);
        double sinChassis = Math.sin(chassisYawRad);
        double turretX = chassisPose2d.getX()
                + (cosChassis * vizTurretPivotOffsetXMeters)
                - (sinChassis * vizTurretPivotOffsetYMeters);
        double turretY = chassisPose2d.getY()
                + (sinChassis * vizTurretPivotOffsetXMeters)
                + (cosChassis * vizTurretPivotOffsetYMeters);

        Pose3d turretPose3d = new Pose3d(
                turretX,
                turretY,
                vizTurretHeightMeters,
                new Rotation3d(0.0, 0.0, turretWorldYawRad)
        );
        Logger.recordOutput("Viz/Robot/TurretPose3d/Valid", 1.0);
        Logger.recordOutput("Viz/Robot/TurretPose3d", turretPose3d);
        Logger.recordOutput("Viz/Robot/TurretRelativeYawDeg", Math.toDegrees(turretRelYawRad));
        Logger.recordOutput("Viz/Robot/TurretWorldYawDeg", Math.toDegrees(turretWorldYawRad));

        if (robot == null || robot.outtake == null || robot.outtake.vision == null || !robot.outtake.vision.hasTarget()) {
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3d/Valid", 0.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dPlusTx/Valid", 0.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dMinusTx/Valid", 0.0);
            return;
        }

        int requiredTagId = robot.outtake.vision.getRequiredTagId();
        double txDeg = robot.outtake.vision.getTxForTag(requiredTagId);
        double distanceMeters = robot.outtake.vision.getDistanceInchesForTag(requiredTagId) * INCHES_TO_METERS;
        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0.0) {
            distanceMeters = vizDefaultTargetRangeMeters;
        }

        double targetYawPlusTxRad = wrapRad(turretWorldYawRad + Math.toRadians(txDeg));
        double targetXPlus = turretX + (Math.cos(targetYawPlusTxRad) * distanceMeters);
        double targetYPlus = turretY + (Math.sin(targetYawPlusTxRad) * distanceMeters);
        Pose3d targetEstimatePose3d = new Pose3d(
            targetXPlus,
            targetYPlus,
            vizTargetHeightMeters,
            new Rotation3d(0.0, 0.0, targetYawPlusTxRad)
        );
        Logger.recordOutput("Viz/Limelight/TargetEstimatePose3d/Valid", 1.0);
        Logger.recordOutput("Viz/Limelight/TargetEstimatePose3d", targetEstimatePose3d);
        Logger.recordOutput("Viz/Limelight/TargetEstimateTxDeg", txDeg);
        Logger.recordOutput("Viz/Limelight/TargetEstimateRangeMeters", distanceMeters);
        Logger.recordOutput("Viz/Limelight/TargetEstimateYawDeg", Math.toDegrees(targetYawPlusTxRad));

        if (vizLogDualTargetSigns) {
            double targetYawMinusTxRad = wrapRad(turretWorldYawRad - Math.toRadians(txDeg));
            double targetXMinus = turretX + (Math.cos(targetYawMinusTxRad) * distanceMeters);
            double targetYMinus = turretY + (Math.sin(targetYawMinusTxRad) * distanceMeters);

            Pose3d targetEstimatePlusTxPose3d = new Pose3d(
                targetXPlus,
                targetYPlus,
                vizTargetHeightMeters,
                new Rotation3d(0.0, 0.0, targetYawPlusTxRad)
            );
            Pose3d targetEstimateMinusTxPose3d = new Pose3d(
                targetXMinus,
                targetYMinus,
                vizTargetHeightMeters,
                new Rotation3d(0.0, 0.0, targetYawMinusTxRad)
            );

            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dPlusTx/Valid", 1.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dPlusTx", targetEstimatePlusTxPose3d);
            Logger.recordOutput("Viz/Limelight/TargetEstimateYawPlusTxDeg", Math.toDegrees(targetYawPlusTxRad));

            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dMinusTx/Valid", 1.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dMinusTx", targetEstimateMinusTxPose3d);
            Logger.recordOutput("Viz/Limelight/TargetEstimateYawMinusTxDeg", Math.toDegrees(targetYawMinusTxRad));
        } else {
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dPlusTx/Valid", 0.0);
            Logger.recordOutput("Viz/Limelight/TargetEstimatePose3dMinusTx/Valid", 0.0);
        }
    }

    private static double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
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

    private static double getPositionAlpha(int tagCount) {
        return (tagCount >= mt1PreferredMultiTagCount) ? mt1PositionAlphaMultiTag : mt1PositionAlphaSingleTag;
    }

    private static double getHeadingAlpha(int tagCount) {
        return (tagCount >= mt1PreferredMultiTagCount) ? mt1HeadingAlphaMultiTag : mt1HeadingAlphaSingleTag;
    }

    private static double getHybridVisionHeadingGain(int tagCount) {
        return (tagCount >= mt1PreferredMultiTagCount) ? hybridVisionHeadingGainMultiTag : hybridVisionHeadingGainSingleTag;
    }

    private static void updateMt1SmoothedState(Pose2d observation, double positionAlpha, double headingAlpha) {
        if (observation == null) {
            return;
        }

        if (!mt1SmoothedState.valid) {
            mt1SmoothedState.set(observation);
            return;
        }

        double jumpMeters = distanceMeters(mt1SmoothedState.toPose2d(), observation);
        double jumpHeadingDeg = Math.toDegrees(angleDifferenceRad(
                mt1SmoothedState.headingRad,
                observation.getRotation().getRadians()
        ));
        if (jumpMeters > mt1StateResetJumpMeters || Math.abs(jumpHeadingDeg) > mt1StateResetHeadingJumpDeg) {
            mt1SmoothedState.set(observation);
            return;
        }

        mt1SmoothedState.xMeters = lerp(mt1SmoothedState.xMeters, observation.getX(), positionAlpha);
        mt1SmoothedState.yMeters = lerp(mt1SmoothedState.yMeters, observation.getY(), positionAlpha);
        mt1SmoothedState.headingRad = angleLerp(mt1SmoothedState.headingRad, observation.getRotation().getRadians(), headingAlpha);
        mt1SmoothedState.valid = true;
    }

    private static double distanceMeters(Pose2d a, Pose2d b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    private static double lerp(double start, double end, double alpha) {
        double clampedAlpha = Math.max(0.0, Math.min(1.0, alpha));
        return start + ((end - start) * clampedAlpha);
    }

    private static double angleLerp(double startRad, double endRad, double alpha) {
        return wrapRad(startRad + (angleDifferenceRad(startRad, endRad) * Math.max(0.0, Math.min(1.0, alpha))));
    }

    private static double angleDifferenceRad(double fromRad, double toRad) {
        return wrapRad(toRad - fromRad);
    }

    private static class CandidatePoseState {
        private boolean valid = false;
        private double xMeters = 0.0;
        private double yMeters = 0.0;
        private double headingRad = 0.0;

        private void set(Pose2d pose) {
            if (pose == null) {
                valid = false;
                return;
            }
            xMeters = pose.getX();
            yMeters = pose.getY();
            headingRad = pose.getRotation().getRadians();
            valid = true;
        }

        private Pose2d toPose2d() {
            return new Pose2d(xMeters, yMeters, Rotation2d.fromRadians(headingRad));
        }
    }
}
