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
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Vision;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "LocalizationPoseLoggerTest", group = "Test")
public class LocalizationPoseLoggerTest extends OpMode {

    private static final double INCHES_TO_METERS = 0.0254;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN * 0.5;

    public static double startXIn = 72.0;
    public static double startYIn = 72.0;
    public static double startHeadingDeg = 30.0; //0.0;
    public static boolean useAllianceDefaultHeading = false;
    public static boolean autoCaptureOffsetsAtStart = true;
    public static boolean logPinpointInFtcCenterRotated = true;
    public static double yawOffsetStepDeg = 1.0;
    public static double yawOffsetFastStepDeg = 5.0;

    private boolean previousA = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousX = false;
    private boolean hasMt2Offset = false;
    private boolean hasBotposeOffset = false;
    private double mt2DxMeters = 0.0;
    private double mt2DyMeters = 0.0;
    private double mt2DHeadingRad = 0.0;
    private double botDxMeters = 0.0;
    private double botDyMeters = 0.0;
    private double botDHeadingRad = 0.0;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

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

        if (autoCaptureOffsetsAtStart) {
            captureOffsetsIfPossible();
        }
    }

    @Override
    public void loop() {
        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }

        applyMecanumDrive();
        handleYawOffsetControls();
        robot.update();

        boolean aPressed = gamepad1.a;
        if (aPressed && !previousA) {
            captureOffsetsIfPossible();
        }
        previousA = aPressed;

        logPoseStreams();
        addTelemetry();
    }

    private void handleYawOffsetControls() {
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean xPressed = gamepad1.x;

        if (dpadLeft && !previousDpadLeft) {
            Vision.robotYawOffsetDeg = AngleUnit.normalizeDegrees(Vision.robotYawOffsetDeg - yawOffsetStepDeg);
        }
        if (dpadRight && !previousDpadRight) {
            Vision.robotYawOffsetDeg = AngleUnit.normalizeDegrees(Vision.robotYawOffsetDeg + yawOffsetStepDeg);
        }
        if (leftBumper && !previousLeftBumper) {
            Vision.robotYawOffsetDeg = AngleUnit.normalizeDegrees(Vision.robotYawOffsetDeg - yawOffsetFastStepDeg);
        }
        if (rightBumper && !previousRightBumper) {
            Vision.robotYawOffsetDeg = AngleUnit.normalizeDegrees(Vision.robotYawOffsetDeg + yawOffsetFastStepDeg);
        }
        if (xPressed && !previousX) {
            Vision.robotYawOffsetDeg = 0.0;
        }

        previousDpadLeft = dpadLeft;
        previousDpadRight = dpadRight;
        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;
        previousX = xPressed;
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

    private void logPoseStreams() {
        Pose followerPose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;
        Pose2d pinpointPedroPose2d = toPose2dFromPedroInches(followerPose);
        Pose2d pinpointFtcPose2d = toPose2dFromPedroInchesAsFtcCenterRotated(followerPose);
        Pose2d pinpointPose2d = logPinpointInFtcCenterRotated ? pinpointFtcPose2d : pinpointPedroPose2d;

        LLResult latest = robot.outtake.vision.latest;
        Pose3D mt2Pose3d = getMt2Pose(latest);
        Pose3D botPose3d = getBotPose(latest);

        Logger.recordOutput("PoseCompare/Pinpoint/Valid", followerPose != null ? 1.0 : 0.0);
        Logger.recordOutput("PoseCompare/Pinpoint/Pose2d", pinpointPose2d);
        Logger.recordOutput("PoseCompare/PinpointPedro/Pose2d", pinpointPedroPose2d);
        Logger.recordOutput("PoseCompare/PinpointFtc/Pose2d", pinpointFtcPose2d);
        if (followerPose != null) {
            Logger.recordOutput("PoseCompare/PinpointRawInches/XIn", followerPose.getX());
            Logger.recordOutput("PoseCompare/PinpointRawInches/YIn", followerPose.getY());
            Logger.recordOutput("PoseCompare/PinpointRawInches/HeadingRad", followerPose.getHeading());
            Logger.recordOutput("PoseCompare/PinpointRawInches/HeadingDeg", Math.toDegrees(followerPose.getHeading()));
        }

        Logger.recordOutput("PoseCompare/Config/StartXIn", startXIn);
        Logger.recordOutput("PoseCompare/Config/StartYIn", startYIn);
        Logger.recordOutput("PoseCompare/Config/StartHeadingDeg", startHeadingDeg);
        Logger.recordOutput("PoseCompare/Config/UseAllianceDefaultHeading", useAllianceDefaultHeading ? 1.0 : 0.0);
        Logger.recordOutput("PoseCompare/Config/LogPinpointAsFtc", logPinpointInFtcCenterRotated ? 1.0 : 0.0);

        Logger.recordOutput("PoseCompare/LimelightMT2/HasTarget", robot.outtake.vision.hasTarget() ? 1.0 : 0.0);
        Logger.recordOutput("PoseCompare/LimelightMT2/Valid", mt2Pose3d != null ? 1.0 : 0.0);
        if (mt2Pose3d != null) {
            Logger.recordOutput("PoseCompare/LimelightMT2/Pose2d", toPose2dFromLimelightMeters(mt2Pose3d));
        }

        Logger.recordOutput("PoseCompare/LimelightBotpose/HasTarget", robot.outtake.vision.hasTarget() ? 1.0 : 0.0);
        Logger.recordOutput("PoseCompare/LimelightBotpose/Valid", botPose3d != null ? 1.0 : 0.0);
        if (botPose3d != null) {
            Logger.recordOutput("PoseCompare/LimelightBotpose/Pose2d", toPose2dFromLimelightMeters(botPose3d));
        }

        Logger.recordOutput("PoseCompare/Align/MT2HasOffset", hasMt2Offset ? 1.0 : 0.0);
        Logger.recordOutput("PoseCompare/Align/BotposeHasOffset", hasBotposeOffset ? 1.0 : 0.0);

        if (followerPose != null && mt2Pose3d != null) {
            Pose2d mt2Raw = toPose2dFromLimelightMeters(mt2Pose3d);
            logRawDelta("PoseCompare/Delta/MT2", pinpointPose2d, mt2Raw);
            if (hasMt2Offset) {
                Logger.recordOutput("PoseCompare/LimelightMT2Aligned/Pose2d", applyOffset(mt2Raw, mt2DxMeters, mt2DyMeters, mt2DHeadingRad));
            }
        }

        if (followerPose != null && botPose3d != null) {
            Pose2d botRaw = toPose2dFromLimelightMeters(botPose3d);
            logRawDelta("PoseCompare/Delta/Botpose", pinpointPose2d, botRaw);
            if (hasBotposeOffset) {
                Logger.recordOutput("PoseCompare/LimelightBotposeAligned/Pose2d", applyOffset(botRaw, botDxMeters, botDyMeters, botDHeadingRad));
            }
        }

        Logger.recordOutput("PoseCompare/Limelight/TagId", robot.outtake.vision.getCurrentTagId());
        Logger.recordOutput("PoseCompare/Limelight/RobotYawOffsetDeg", Vision.robotYawOffsetDeg);
        Logger.recordOutput("PoseCompare/Limelight/ChassisYawDeg", robot.outtake.vision.getLastChassisYawDeg());
        Logger.recordOutput("PoseCompare/Limelight/TurretRelativeYawDeg", robot.outtake.vision.getLastTurretRelativeYawDeg());
        Logger.recordOutput("PoseCompare/Limelight/RobotYawSentDeg", robot.outtake.vision.getLastRobotYawSentDeg());
        Logger.recordOutput("PoseCompare/Limelight/RobotYawSendSuccess", robot.outtake.vision.wasLastRobotYawSendSuccessful() ? 1.0 : 0.0);
    }

    private void logRawDelta(String keyPrefix, Pose2d reference, Pose2d measured) {
        double dx = reference.getX() - measured.getX();
        double dy = reference.getY() - measured.getY();
        double dTheta = wrapRad(reference.getRotation().getRadians() - measured.getRotation().getRadians());
        Logger.recordOutput(keyPrefix + "/DxMeters", dx);
        Logger.recordOutput(keyPrefix + "/DyMeters", dy);
        Logger.recordOutput(keyPrefix + "/DistMeters", Math.hypot(dx, dy));
        Logger.recordOutput(keyPrefix + "/DHeadingDeg", Math.toDegrees(dTheta));
    }

    private Pose2d applyOffset(Pose2d raw, double dx, double dy, double dHeadingRad) {
        return new Pose2d(
                raw.getX() + dx,
                raw.getY() + dy,
                Rotation2d.fromRadians(raw.getRotation().getRadians() + dHeadingRad)
        );
    }

    private void captureOffsetsIfPossible() {
        Pose followerPose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;
        if (followerPose == null) {
            return;
        }

        Pose2d pinpointPose2d = toPose2dFromPedroInches(followerPose);
        if (logPinpointInFtcCenterRotated) {
            pinpointPose2d = toPose2dFromPedroInchesAsFtcCenterRotated(followerPose);
        }
        LLResult latest = robot.outtake.vision.latest;
        Pose3D mt2Pose3d = getMt2Pose(latest);
        Pose3D botPose3d = getBotPose(latest);

        if (mt2Pose3d != null) {
            Pose2d mt2Pose2d = toPose2dFromLimelightMeters(mt2Pose3d);
            mt2DxMeters = pinpointPose2d.getX() - mt2Pose2d.getX();
            mt2DyMeters = pinpointPose2d.getY() - mt2Pose2d.getY();
            mt2DHeadingRad = wrapRad(pinpointPose2d.getRotation().getRadians() - mt2Pose2d.getRotation().getRadians());
            hasMt2Offset = true;
        }

        if (botPose3d != null) {
            Pose2d botPose2d = toPose2dFromLimelightMeters(botPose3d);
            botDxMeters = pinpointPose2d.getX() - botPose2d.getX();
            botDyMeters = pinpointPose2d.getY() - botPose2d.getY();
            botDHeadingRad = wrapRad(pinpointPose2d.getRotation().getRadians() - botPose2d.getRotation().getRadians());
            hasBotposeOffset = true;
        }
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private Pose3D getMt2Pose(LLResult latest) {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose_MT2();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private Pose3D getBotPose(LLResult latest) {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private Pose2d toPose2dFromPedroInches(Pose pedroPose) {
        if (pedroPose == null) {
            return Pose2d.kZero;
        }

        double xMeters = pedroPose.getX() * INCHES_TO_METERS;
        double yMeters = pedroPose.getY() * INCHES_TO_METERS;
        return new Pose2d(xMeters, yMeters, Rotation2d.fromRadians(pedroPose.getHeading()));
    }

    private Pose2d toPose2dFromPedroInchesAsFtcCenterRotated(Pose pedroPose) {
        if (pedroPose == null) {
            return Pose2d.kZero;
        }

        double pedroXIn = pedroPose.getX();
        double pedroYIn = pedroPose.getY();
        double pedroHeadingRad = pedroPose.getHeading();

        // Pedro frame: origin bottom-left, +X right, +Y up.
        // FTC Center/Rotated: origin center, +X toward red wall (screen-down in docs), +Y to the right.
        double ftcXIn = FIELD_HALF_IN - pedroYIn;
        double ftcYIn = pedroXIn - FIELD_HALF_IN;
        double ftcHeadingRad = wrapRad(pedroHeadingRad + (Math.PI * 0.5));

        return new Pose2d(
                ftcXIn * INCHES_TO_METERS,
                ftcYIn * INCHES_TO_METERS,
                Rotation2d.fromRadians(ftcHeadingRad)
        );
    }

    private Pose2d toPose2dFromLimelightMeters(Pose3D llPose) {
        double xMeters = llPose.getPosition().x;
        double yMeters = llPose.getPosition().y;
        double headingRad = Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES));
        return new Pose2d(xMeters, yMeters, Rotation2d.fromRadians(headingRad));
    }

    private void addTelemetry() {
        Pose followerPose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;
        LLResult latest = robot.outtake.vision.latest;
        Pose3D mt2Pose3d = getMt2Pose(latest);
        Pose3D botPose3d = getBotPose(latest);

        joinedTelemetry.addData("Drive Controls", "Left stick translate, right stick rotate");
        joinedTelemetry.addData("Offset Capture", "Press A to capture Pinpoint->LL offsets");
        joinedTelemetry.addData("Yaw Offset Ctl", "Dpad L/R=±1, LB/RB=±5, X=reset");
        joinedTelemetry.addData("Pinpoint Frame", logPinpointInFtcCenterRotated ? "FTC Center/Rotated" : "Pedro Bottom-Left");
        joinedTelemetry.addData("LL Has Target", robot.outtake.vision.hasTarget());
        joinedTelemetry.addData("LL Tag ID", robot.outtake.vision.getCurrentTagId());
        joinedTelemetry.addData("LL Yaw Offset (deg)", "%.1f", Vision.robotYawOffsetDeg);
        joinedTelemetry.addData("LL Chassis Yaw (deg)", "%.1f", robot.outtake.vision.getLastChassisYawDeg());
        joinedTelemetry.addData("LL Turret Rel Yaw (deg)", "%.1f", robot.outtake.vision.getLastTurretRelativeYawDeg());
        joinedTelemetry.addData("LL Yaw Sent (deg)", "%.1f", robot.outtake.vision.getLastRobotYawSentDeg());
        joinedTelemetry.addData("LL Yaw Send OK", robot.outtake.vision.wasLastRobotYawSendSuccessful());
        joinedTelemetry.addData("MT2 Offset Captured", hasMt2Offset);
        joinedTelemetry.addData("Botpose Offset Captured", hasBotposeOffset);
        joinedTelemetry.addData("Pinpoint Valid", followerPose != null);
        if (followerPose != null) {
            joinedTelemetry.addData("Pinpoint (in)", "x=%.1f y=%.1f h=%.1f",
                    followerPose.getX(),
                    followerPose.getY(),
                    Math.toDegrees(followerPose.getHeading()));
        }

        joinedTelemetry.addData("MT2 Valid", mt2Pose3d != null);
        if (mt2Pose3d != null) {
            joinedTelemetry.addData("MT2 (m)", "x=%.3f y=%.3f h=%.1f",
                    mt2Pose3d.getPosition().x,
                    mt2Pose3d.getPosition().y,
                    mt2Pose3d.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        joinedTelemetry.addData("Botpose Valid", botPose3d != null);
        if (botPose3d != null) {
            joinedTelemetry.addData("Botpose (m)", "x=%.3f y=%.3f h=%.1f",
                    botPose3d.getPosition().x,
                    botPose3d.getPosition().y,
                    botPose3d.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        joinedTelemetry.update();
    }
}
