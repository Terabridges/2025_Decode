package org.firstinspires.ftc.teamcode.config.utility;

import com.bylazar.configurables.annotations.Configurable;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

@Configurable
public class LocalizationCandidateCalculator {

    public static double mt1FieldOffsetXMeters = 0.0;
    public static double mt1FieldOffsetYMeters = 0.0;
    public static double mt1FieldOffsetHeadingDeg = 0.0;
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

    private final CandidatePoseState mt1SmoothedState = new CandidatePoseState();

    public CandidateResult update(Pose2d pinpointFtcPose, Pose2d mt1PoseComp, int tagCount,
                                  double planarDistanceIn, double robotSpeedInS) {
        CandidateResult result = new CandidateResult();
        result.mt1CalibratedPose = applyMt1Calibration(mt1PoseComp);
        result.tagCount = tagCount;
        result.planarDistanceIn = planarDistanceIn;
        result.robotSpeedInS = robotSpeedInS;

        result.rejectNoObservation = (result.mt1CalibratedPose == null);
        result.rejectTagCount = !result.rejectNoObservation && tagCount < mt1MinTagCount;
        result.rejectDistance = !result.rejectNoObservation && Double.isFinite(planarDistanceIn) && planarDistanceIn > mt1MaxPlanarDistanceIn;
        result.rejectRobotSpeed = !result.rejectNoObservation && Double.isFinite(robotSpeedInS) && robotSpeedInS > mt1MaxRobotSpeedInS;

        if (pinpointFtcPose != null && result.mt1CalibratedPose != null) {
            result.translationErrorMeters = distanceMeters(pinpointFtcPose, result.mt1CalibratedPose);
            result.headingErrorDeg = Math.toDegrees(angleDifferenceRad(
                    pinpointFtcPose.getRotation().getRadians(),
                result.mt1CalibratedPose.getRotation().getRadians()
            ));
            result.rejectTranslationJump = result.translationErrorMeters > mt1MaxTranslationJumpMeters;
            result.rejectHeadingJump = Math.abs(result.headingErrorDeg) > mt1MaxHeadingJumpDeg;
        } else {
            result.translationErrorMeters = Double.NaN;
            result.headingErrorDeg = Double.NaN;
        }

        result.accepted = !result.rejectNoObservation
            && !result.rejectTagCount
            && !result.rejectDistance
            && !result.rejectRobotSpeed
            && !result.rejectTranslationJump
            && !result.rejectHeadingJump;
        result.translationAccepted = result.accepted;
        result.headingAccepted = result.accepted;

        if (result.accepted && result.mt1CalibratedPose != null) {
            result.positionAlpha = getPositionAlpha(tagCount);
            result.headingAlpha = getHeadingAlpha(tagCount);
            updateMt1SmoothedState(result.mt1CalibratedPose, result.positionAlpha, result.headingAlpha, true);
        }

        result.smoothedValid = mt1SmoothedState.valid;
        if (mt1SmoothedState.valid) {
            result.mt1SmoothedPose = mt1SmoothedState.toPose2d();
            if (pinpointFtcPose != null) {
                result.hybridPinpointHeadingPose = new Pose2d(
                        lerp(pinpointFtcPose.getX(), result.mt1SmoothedPose.getX(), hybridTranslationGain),
                        lerp(pinpointFtcPose.getY(), result.mt1SmoothedPose.getY(), hybridTranslationGain),
                        Rotation2d.fromRadians(pinpointFtcPose.getRotation().getRadians())
                );

                result.hybridVisionHeadingGain = getHybridVisionHeadingGain(tagCount);
                result.hybridVisionHeadingPose = new Pose2d(
                        result.hybridPinpointHeadingPose.getX(),
                        result.hybridPinpointHeadingPose.getY(),
                        Rotation2d.fromRadians(angleLerp(
                                pinpointFtcPose.getRotation().getRadians(),
                                result.mt1SmoothedPose.getRotation().getRadians(),
                                result.hybridVisionHeadingGain
                        ))
                );
                result.pinpointVsSmoothedTranslationErrorM = distanceMeters(pinpointFtcPose, result.mt1SmoothedPose);
                result.pinpointVsSmoothedHeadingErrorDeg = Math.toDegrees(angleDifferenceRad(
                        pinpointFtcPose.getRotation().getRadians(),
                        result.mt1SmoothedPose.getRotation().getRadians()
                ));
            }
        }

        return result;
    }

    public void recordOutputs(String basePrefix, CandidateResult result) {
        Logger.recordOutput(basePrefix + "/MT1Smoothed/Accepted", result.accepted ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/MT1Smoothed/TranslationAccepted", result.translationAccepted ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/MT1Smoothed/HeadingAccepted", result.headingAccepted ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/MT1Smoothed/Valid", result.smoothedValid ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/MT1Calibrated/Valid", result.mt1CalibratedPose != null ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/Diagnostics/TagCount", result.tagCount);
        Logger.recordOutput(basePrefix + "/Diagnostics/PlanarDistanceIn", result.planarDistanceIn);
        Logger.recordOutput(basePrefix + "/Diagnostics/RobotSpeedInS", result.robotSpeedInS);
        Logger.recordOutput(basePrefix + "/Diagnostics/Calibration/XMeters", mt1FieldOffsetXMeters);
        Logger.recordOutput(basePrefix + "/Diagnostics/Calibration/YMeters", mt1FieldOffsetYMeters);
        Logger.recordOutput(basePrefix + "/Diagnostics/Calibration/HeadingDeg", mt1FieldOffsetHeadingDeg);
        Logger.recordOutput(basePrefix + "/Diagnostics/PinpointVsMT1TranslationErrorM", result.translationErrorMeters);
        Logger.recordOutput(basePrefix + "/Diagnostics/PinpointVsMT1HeadingErrorDeg", result.headingErrorDeg);
        Logger.recordOutput(basePrefix + "/Diagnostics/Reject/NoObservation", result.rejectNoObservation ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/Diagnostics/Reject/TagCount", result.rejectTagCount ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/Diagnostics/Reject/Distance", result.rejectDistance ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/Diagnostics/Reject/RobotSpeed", result.rejectRobotSpeed ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/Diagnostics/Reject/TranslationJump", result.rejectTranslationJump ? 1.0 : 0.0);
        Logger.recordOutput(basePrefix + "/Diagnostics/Reject/HeadingJump", result.rejectHeadingJump ? 1.0 : 0.0);

        if (Double.isFinite(result.positionAlpha)) {
            Logger.recordOutput(basePrefix + "/Diagnostics/PositionAlpha", result.positionAlpha);
        }
        if (Double.isFinite(result.headingAlpha)) {
            Logger.recordOutput(basePrefix + "/Diagnostics/HeadingAlpha", result.headingAlpha);
        }
        if (result.mt1CalibratedPose != null) {
            Logger.recordOutput(basePrefix + "/MT1Calibrated/Pose2d", result.mt1CalibratedPose);
        }
        if (result.mt1SmoothedPose != null) {
            Logger.recordOutput(basePrefix + "/MT1Smoothed/Pose2d", result.mt1SmoothedPose);
        }
        if (result.hybridPinpointHeadingPose != null) {
            Logger.recordOutput(basePrefix + "/HybridPinpointHeading/Pose2d", result.hybridPinpointHeadingPose);
        }
        if (result.hybridVisionHeadingPose != null) {
            Logger.recordOutput(basePrefix + "/HybridVisionHeading/Pose2d", result.hybridVisionHeadingPose);
            Logger.recordOutput(basePrefix + "/Diagnostics/HybridVisionHeadingGain", result.hybridVisionHeadingGain);
        }
        if (Double.isFinite(result.pinpointVsSmoothedTranslationErrorM)) {
            Logger.recordOutput(basePrefix + "/Diagnostics/PinpointVsSmoothedTranslationErrorM", result.pinpointVsSmoothedTranslationErrorM);
        }
        if (Double.isFinite(result.pinpointVsSmoothedHeadingErrorDeg)) {
            Logger.recordOutput(basePrefix + "/Diagnostics/PinpointVsSmoothedHeadingErrorDeg", result.pinpointVsSmoothedHeadingErrorDeg);
        }
    }

    public void reset() {
        mt1SmoothedState.valid = false;
    }

    public static Pose2d applyMt1Calibration(Pose2d mt1Pose) {
        if (mt1Pose == null) {
            return null;
        }

        return new Pose2d(
                mt1Pose.getX() + mt1FieldOffsetXMeters,
                mt1Pose.getY() + mt1FieldOffsetYMeters,
                Rotation2d.fromRadians(wrapRad(
                        mt1Pose.getRotation().getRadians() + Math.toRadians(mt1FieldOffsetHeadingDeg)
                ))
        );
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

    private void updateMt1SmoothedState(Pose2d observation, double positionAlpha, double headingAlpha, boolean updateHeading) {
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
        if (updateHeading) {
            mt1SmoothedState.headingRad = angleLerp(mt1SmoothedState.headingRad, observation.getRotation().getRadians(), headingAlpha);
        }
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

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
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

    public static class CandidateResult {
        public int tagCount;
        public double planarDistanceIn = Double.NaN;
        public double robotSpeedInS = Double.NaN;
        public boolean rejectNoObservation;
        public boolean rejectTagCount;
        public boolean rejectDistance;
        public boolean rejectRobotSpeed;
        public boolean rejectTranslationJump;
        public boolean rejectHeadingJump;
        public boolean accepted;
        public boolean translationAccepted;
        public boolean headingAccepted;
        public boolean smoothedValid;
        public double translationErrorMeters = Double.NaN;
        public double headingErrorDeg = Double.NaN;
        public double positionAlpha = Double.NaN;
        public double headingAlpha = Double.NaN;
        public double hybridVisionHeadingGain = Double.NaN;
        public double pinpointVsSmoothedTranslationErrorM = Double.NaN;
        public double pinpointVsSmoothedHeadingErrorDeg = Double.NaN;
        public Pose2d mt1CalibratedPose;
        public Pose2d mt1SmoothedPose;
        public Pose2d hybridPinpointHeadingPose;
        public Pose2d hybridVisionHeadingPose;
    }
}