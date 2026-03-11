package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Vision;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.LocalizationBiasFileStore;
import org.firstinspires.ftc.teamcode.config.utility.LocalizationCandidateCalculator;
import org.firstinspires.ftc.teamcode.config.utility.PoseLoggingUtil;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.io.IOException;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "LocalizationBiasCapture", group = "Test")
public class LocalizationBiasCapture extends OpMode {

    private static final double INCHES_TO_METERS = 0.0254;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN * 0.5;

    public static double startXIn = 72.0;
    public static double startYIn = 72.0;
    public static double startHeadingDeg = 0.0;
    public static boolean useAllianceDefaultHeading = true;
    public static boolean autoLoadStoredBiasAtStart = true;
    public static boolean autoApplyCapturedBias = true;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousLeftBumper = false;

    private LocalizationBiasFileStore.Bias capturedBias = LocalizationBiasFileStore.Bias.invalid();
    private LocalizationBiasFileStore.Bias storedBias = LocalizationBiasFileStore.Bias.invalid();
    private String lastStatus = "Init";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        robot.toInit();
        robot.other.drive.manualDrive = true;
        robot.outtake.setAimLockEnabled(false);
    }

    @Override
    public void start() {
        double headingRad = Math.toRadians(startHeadingDeg);
        if (useAllianceDefaultHeading) {
            headingRad = GlobalVariables.isBlueAlliance() ? Math.PI : 0.0;
        }
        FollowerManager.initFollower(hardwareMap, new Pose(startXIn, startYIn, headingRad));

        if (autoLoadStoredBiasAtStart) {
            loadStoredBiasForCurrentAlliance(true);
        }
    }

    @Override
    public void loop() {
        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }

        applyMecanumDrive();
        robot.update();
        PoseLoggingUtil.logMainPoseDetails(robot);

        handleControls();
        logBiasCaptureData();
        addTelemetry();
    }

    private void handleControls() {
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;
        boolean xPressed = gamepad1.x;
        boolean yPressed = gamepad1.y;
        boolean leftBumper = gamepad1.left_bumper;

        if (aPressed && !previousA) {
            captureBias();
        }
        if (xPressed && !previousX) {
            loadStoredBiasForCurrentAlliance(false);
        }
        if (yPressed && !previousY) {
            saveCurrentBiasForCurrentAlliance();
        }
        if (bPressed && !previousB) {
            GlobalVariables.toggleAlliance();
            lastStatus = "Alliance toggled to " + GlobalVariables.getAllianceColorName();
            loadStoredBiasForCurrentAlliance(false);
        }
        if (leftBumper && !previousLeftBumper) {
            LocalizationCandidateCalculator.mt1FieldOffsetXMeters = 0.0;
            LocalizationCandidateCalculator.mt1FieldOffsetYMeters = 0.0;
            LocalizationCandidateCalculator.mt1FieldOffsetHeadingDeg = 0.0;
            lastStatus = "Current applied bias reset to zero";
        }

        previousA = aPressed;
        previousB = bPressed;
        previousX = xPressed;
        previousY = yPressed;
        previousLeftBumper = leftBumper;
    }

    private void captureBias() {
        Pose2d pinpointPose = getPinpointPose();
        Pose2d mt1Pose = getMt1Pose();
        if (pinpointPose == null || mt1Pose == null) {
            lastStatus = "Capture failed: need Pinpoint + MT1 valid";
            return;
        }

        capturedBias = LocalizationBiasFileStore.captureBias(pinpointPose, mt1Pose);
        if (autoApplyCapturedBias) {
            LocalizationBiasFileStore.applyBiasToCalculator(capturedBias);
            lastStatus = "Captured and applied bias for " + GlobalVariables.getAllianceColorName();
        } else {
            lastStatus = "Captured bias for " + GlobalVariables.getAllianceColorName();
        }
    }

    private void loadStoredBiasForCurrentAlliance(boolean quietIfMissing) {
        try {
            storedBias = LocalizationBiasFileStore.loadAllianceBias(GlobalVariables.getAllianceColor());
            if (storedBias.valid) {
                LocalizationBiasFileStore.applyBiasToCalculator(storedBias);
                lastStatus = "Loaded stored bias for " + GlobalVariables.getAllianceColorName();
            } else if (!quietIfMissing) {
                lastStatus = "No stored bias for " + GlobalVariables.getAllianceColorName();
            }
        } catch (IOException e) {
            lastStatus = "Load failed: " + e.getMessage();
        }
    }

    private void saveCurrentBiasForCurrentAlliance() {
        try {
            LocalizationBiasFileStore.saveAllianceBias(
                    GlobalVariables.getAllianceColor(),
                    LocalizationBiasFileStore.getCalculatorBias()
            );
            storedBias = LocalizationBiasFileStore.getCalculatorBias();
            lastStatus = "Saved bias for " + GlobalVariables.getAllianceColorName();
        } catch (IOException e) {
            lastStatus = "Save failed: " + e.getMessage();
        }
    }

    private void logBiasCaptureData() {
        Pose2d pinpointPose = getPinpointPose();
        Pose2d mt1Pose = getMt1Pose();
        Pose2d currentCalibratedPose = LocalizationCandidateCalculator.applyMt1Calibration(mt1Pose);
        Pose2d capturedPose = LocalizationBiasFileStore.applyBias(mt1Pose, capturedBias);
        Pose2d storedPose = LocalizationBiasFileStore.applyBias(mt1Pose, storedBias);
        LocalizationBiasFileStore.Bias currentBias = LocalizationBiasFileStore.getCalculatorBias();

        Logger.recordOutput("Localization/BiasCapture/Alliance", GlobalVariables.getAllianceColorName());
        Logger.recordOutput("Localization/BiasCapture/StorePath", LocalizationBiasFileStore.getStorePath());
        Logger.recordOutput("Localization/BiasCapture/Captured/Valid", capturedBias.valid ? 1.0 : 0.0);
        Logger.recordOutput("Localization/BiasCapture/Stored/Valid", storedBias.valid ? 1.0 : 0.0);
        Logger.recordOutput("Localization/BiasCapture/CurrentApplied/Valid", 1.0);
        Logger.recordOutput("Localization/BiasCapture/CurrentApplied/XMeters", currentBias.xMeters);
        Logger.recordOutput("Localization/BiasCapture/CurrentApplied/YMeters", currentBias.yMeters);
        Logger.recordOutput("Localization/BiasCapture/CurrentApplied/HeadingDeg", currentBias.headingDeg);
        Logger.recordOutput("Localization/BiasCapture/Captured/XMeters", capturedBias.xMeters);
        Logger.recordOutput("Localization/BiasCapture/Captured/YMeters", capturedBias.yMeters);
        Logger.recordOutput("Localization/BiasCapture/Captured/HeadingDeg", capturedBias.headingDeg);
        Logger.recordOutput("Localization/BiasCapture/Stored/XMeters", storedBias.xMeters);
        Logger.recordOutput("Localization/BiasCapture/Stored/YMeters", storedBias.yMeters);
        Logger.recordOutput("Localization/BiasCapture/Stored/HeadingDeg", storedBias.headingDeg);

        if (pinpointPose != null) {
            Logger.recordOutput("Localization/BiasCapture/Reference/Pose2d", pinpointPose);
        }
        if (mt1Pose != null) {
            Logger.recordOutput("Localization/BiasCapture/MT1Raw/Pose2d", mt1Pose);
        }
        if (currentCalibratedPose != null) {
            Logger.recordOutput("Localization/BiasCapture/MT1CurrentCalibrated/Pose2d", currentCalibratedPose);
        }
        if (capturedPose != null) {
            Logger.recordOutput("Localization/BiasCapture/MT1CapturedBias/Pose2d", capturedPose);
        }
        if (storedPose != null) {
            Logger.recordOutput("Localization/BiasCapture/MT1StoredBias/Pose2d", storedPose);
        }
    }

    private void addTelemetry() {
        LocalizationBiasFileStore.Bias currentBias = LocalizationBiasFileStore.getCalculatorBias();
        joinedTelemetry.addData("Alliance", GlobalVariables.getAllianceColorName());
        joinedTelemetry.addData("Controls", "A capture | X load | Y save | B toggle alliance | LB reset current");
        joinedTelemetry.addData("Drive", "Left stick translate, right stick rotate");
        joinedTelemetry.addData("Store File", LocalizationBiasFileStore.getStorePath());
        joinedTelemetry.addData("Status", lastStatus);
        joinedTelemetry.addData("MT1 Valid", getMt1Pose() != null);
        joinedTelemetry.addData("Pinpoint Valid", getPinpointPose() != null);
        joinedTelemetry.addData("Auto Apply Capture", autoApplyCapturedBias);
        joinedTelemetry.addData("Captured Valid", capturedBias.valid);
        joinedTelemetry.addData("Stored Valid", storedBias.valid);
        joinedTelemetry.addData("Captured Bias (m)", "x=%.3f y=%.3f h=%.1f", capturedBias.xMeters, capturedBias.yMeters, capturedBias.headingDeg);
        joinedTelemetry.addData("Stored Bias (m)", "x=%.3f y=%.3f h=%.1f", storedBias.xMeters, storedBias.yMeters, storedBias.headingDeg);
        joinedTelemetry.addData("Current Bias (m)", "x=%.3f y=%.3f h=%.1f", currentBias.xMeters, currentBias.yMeters, currentBias.headingDeg);
        joinedTelemetry.addData("Current Bias (in)", "x=%.1f y=%.1f", currentBias.xMeters / INCHES_TO_METERS, currentBias.yMeters / INCHES_TO_METERS);
        joinedTelemetry.update();
    }

    private void applyMecanumDrive() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        robot.other.drive.setDrivePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private Pose2d getPinpointPose() {
        Pose followerPose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;
        if (followerPose == null) {
            return null;
        }

        double pedroXIn = followerPose.getX();
        double pedroYIn = followerPose.getY();
        double pedroHeadingRad = followerPose.getHeading();

        double ftcXIn = FIELD_HALF_IN - pedroYIn;
        double ftcYIn = pedroXIn - FIELD_HALF_IN;
        double ftcHeadingRad = wrapRad(pedroHeadingRad + (Math.PI * 0.5));

        return new Pose2d(
                ftcXIn * INCHES_TO_METERS,
                ftcYIn * INCHES_TO_METERS,
                Rotation2d.fromRadians(ftcHeadingRad)
        );
    }

    private Pose2d getMt1Pose() {
        LLResult latest = robot.outtake.vision.latest;
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            Pose3D mt1Pose = latest.getBotpose();
            if (mt1Pose == null) {
                return null;
            }
            double[] corrected = robot.outtake.vision.getTurretCompensatedPose2dMetersFromPose3d(mt1Pose);
            if (corrected == null || corrected.length < 3) {
                return new Pose2d(
                        mt1Pose.getPosition().x,
                        mt1Pose.getPosition().y,
                        Rotation2d.fromRadians(Math.toRadians(mt1Pose.getOrientation().getYaw(AngleUnit.DEGREES)))
                );
            }
            return new Pose2d(
                    corrected[0],
                    corrected[1],
                    Rotation2d.fromRadians(Math.toRadians(corrected[2]))
            );
        } catch (Throwable ignored) {
            return null;
        }
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
