package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.control.Other.DriveControl;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

@Configurable
@TeleOp(name = "MT2RelocalizationDriveTest", group = "Test")
public class MT2RelocalizationDriveTest extends OpMode {
    private static final double METERS_TO_INCHES = 39.3701;

    public static double startX = 72.0;
    public static double startY = 72.0;
    public static double startHeadingDeg = 0.0; // red-side teleop heading
    public static boolean requireRedGoalTag = false;
    public static boolean applyRelocalizeOnPress = true;

    private Robot robot;
    private DriveControl driveControl;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    private boolean lastAttemptSuccess = false;
    private String lastAttemptReason = "N/A";
    private Pose beforePose = null;
    private Pose afterPose = null;
    private int lastSeenGoalTag = -1;

    @Override
    public void init() {
        GlobalVariables.setAllianceColor(GlobalVariables.AllianceColor.RED);

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        driveControl = new DriveControl(robot, gamepad1, gamepad2);
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        robot.toInit();
        robot.outtake.turret.setAimLockEnabled(false);
        FollowerManager.initFollower(hardwareMap, new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (follower != null) {
            follower.update();
        }

        driveControl.update();
        robot.update();

        if (currentGamepad1.b && !previousGamepad1.b) {
            attemptRelocalization();
        }

        drawCurrentAndHistory();
        addTelemetry();
    }

    private void attemptRelocalization() {
        beforePose = copyPose(follower != null ? follower.getPose() : null);

        if (follower == null || follower.getPose() == null) {
            lastAttemptSuccess = false;
            lastAttemptReason = "Follower pose unavailable";
            return;
        }

        Pose followerPose = follower.getPose();
        robot.outtake.vision.updateRobotYawDegrees(Math.toDegrees(followerPose.getHeading()));

        int visibleTag = robot.outtake.vision.getVisibleGoalTagId();
        lastSeenGoalTag = visibleTag;
        if (visibleTag < 0) {
            lastAttemptSuccess = false;
            lastAttemptReason = "No goal tag (20/24) in frame";
            return;
        }
        if (requireRedGoalTag && visibleTag != 24) {
            lastAttemptSuccess = false;
            lastAttemptReason = "Visible goal tag is not RED(24)";
            return;
        }

        Pose3D mt2 = robot.outtake.vision.getLatestBotPoseMT2();
        if (mt2 == null) {
            lastAttemptSuccess = false;
            lastAttemptReason = "MT2 pose unavailable";
            return;
        }

        double xIn = mt2.getPosition().x * METERS_TO_INCHES;
        double yIn = mt2.getPosition().y * METERS_TO_INCHES;
        double headingRad = Math.toRadians(mt2.getOrientation().getYaw(AngleUnit.DEGREES));

        if (applyRelocalizeOnPress) {
            follower.setPose(new Pose(xIn, yIn, headingRad));
        }

        afterPose = copyPose(follower != null ? follower.getPose() : null);
        lastAttemptSuccess = true;
        lastAttemptReason = applyRelocalizeOnPress ? "Applied MT2 relocalization" : "MT2 pose sampled only";
    }

    private void addTelemetry() {
        Pose followerPose = (follower != null) ? follower.getPose() : null;
        LLResult latest = robot.outtake.vision.latest;

        telemetry.addData("Controls", "gp1 sticks drive, gp1.B attempt MT2 relocalize");
        telemetry.addData("Alliance", GlobalVariables.getAllianceColorName());
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.1fdeg)", startX, startY, startHeadingDeg);
        telemetry.addData("Apply On Press", applyRelocalizeOnPress);
        telemetry.addData("Require RED Tag", requireRedGoalTag);

        telemetry.addData("Turret Cmd (deg)", "%.1f", robot.outtake.turret.getCurrentDegrees());
        telemetry.addData("Turret->Forward Err (deg)", "%.1f",
                wrapSignedDeg(Turret.turretForwardDeg - robot.outtake.turret.getCurrentDegrees()));

        telemetry.addData("Visible Goal Tag", robot.outtake.vision.getVisibleGoalTagId());
        telemetry.addData("Sees Red(24)", robot.outtake.vision.seesTag(24));
        telemetry.addData("Sees Blue(20)", robot.outtake.vision.seesTag(20));
        telemetry.addData("Tx", "%.2f", robot.outtake.vision.getTx());
        telemetry.addData("Ty", "%.2f", robot.outtake.vision.getTy());
        telemetry.addData("Distance (in)", "%.2f", robot.outtake.vision.getDistanceInches());

        if (latest != null) {
            telemetry.addData("LL Staleness (ms)", latest.getStaleness());
            telemetry.addData("LL Capture Lat (ms)", "%.1f", latest.getCaptureLatency());
            telemetry.addData("LL Target Lat (ms)", "%.1f", latest.getTargetingLatency());
            telemetry.addData("LL Parse Lat (ms)", "%.1f", latest.getParseLatency());
        } else {
            telemetry.addData("LL", "No latest result");
        }

        Pose3D mt2 = robot.outtake.vision.getLatestBotPoseMT2();
        if (mt2 != null) {
            telemetry.addData("MT2 X(in)", "%.2f", mt2.getPosition().x * METERS_TO_INCHES);
            telemetry.addData("MT2 Y(in)", "%.2f", mt2.getPosition().y * METERS_TO_INCHES);
            telemetry.addData("MT2 Heading(deg)", "%.1f", mt2.getOrientation().getYaw(AngleUnit.DEGREES));
        } else {
            telemetry.addData("MT2", "Unavailable");
        }

        if (followerPose != null) {
            telemetry.addData("Follower X", "%.2f", followerPose.getX());
            telemetry.addData("Follower Y", "%.2f", followerPose.getY());
            telemetry.addData("Follower H(deg)", "%.1f", Math.toDegrees(followerPose.getHeading()));
        } else {
            telemetry.addData("Follower Pose", "null");
        }

        telemetry.addData("Last Attempt Success", lastAttemptSuccess);
        telemetry.addData("Last Attempt Reason", lastAttemptReason);
        telemetry.addData("Last Seen Goal Tag", lastSeenGoalTag);
        addPoseTelemetry("Before", beforePose);
        addPoseTelemetry("After", afterPose);
        telemetry.update();
    }

    private void addPoseTelemetry(String label, Pose pose) {
        if (pose == null) {
            telemetry.addData(label, "null");
            return;
        }
        telemetry.addData(label, "x=%.2f y=%.2f h=%.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private double wrapSignedDeg(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
