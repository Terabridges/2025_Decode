package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Relocalization.GoalTagRelocalization;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

@Configurable
@TeleOp(name = "GoalTagRelocDriveTester", group = "Test")
public class GoalTagRelocalizationDriveTester extends OpMode {

    public static double driveScale = 1.0;
    public static double turnScale = 1.0;
    public static boolean useFieldCentric = false;

    private Robot robot;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Pose blueLongStartPose;
    private GoalTagRelocalization.GoalTagRelocalizeResult lastRelocalizeResult;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        robot.toInit();
        robot.outtake.turret.setAimLockEnabled(false);

        AutoPoses poses = new AutoPoses();
        Pose p = poses.blueFarStartPose;
        blueLongStartPose = new Pose(p.getX(), p.getY(), p.getHeading());
        FollowerManager.initFollower(hardwareMap, blueLongStartPose);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (follower != null) {
            follower.update();
        }

        handleRelocalizationButtons();
        updateDriveFromGamepad();
        robot.update();

        lastRelocalizeResult = robot.relocalization.getLatestGoalTagRelocalizeResult();

        drawCurrentAndHistory();
        addTelemetry();
    }

    private void handleRelocalizationButtons() {
        if (currentGamepad1.b && !previousGamepad1.b) {
            // Turret servos are currently unreliable, so do not command a fixed forward target here.
            // We prime/execute using the current measured command and let driver manually place turret forward.
            double holdCurrentDeg = robot.outtake.turret.getCurrentDegrees();
            robot.relocalization.stepGoalTagRelocalization(holdCurrentDeg);
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            if (follower != null) {
                follower.setPose(new Pose(blueLongStartPose.getX(), blueLongStartPose.getY(), blueLongStartPose.getHeading()));
            }
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            robot.relocalization.cancelGoalTagRelocalizationPrime();
            robot.relocalization.clearGoalTagRelocalizeResult();
            lastRelocalizeResult = null;
        }
    }

    private void updateDriveFromGamepad() {
        robot.other.drive.manualDrive = true;
        robot.other.drive.useFieldCentric = useFieldCentric;

        double forward = -gamepad1.left_stick_y * driveScale;
        double right = gamepad1.left_stick_x * driveScale;
        double rotate = gamepad1.right_stick_x * turnScale;

        if (useFieldCentric && follower != null) {
            robot.other.drive.driveFieldRelative(forward, right, rotate);
        } else {
            robot.other.drive.drive(forward, right, rotate);
        }
    }

    private void addTelemetry() {
        Pose followerPose = (follower != null) ? follower.getPose() : null;
        int visibleGoalTag = robot.outtake.vision.getVisibleGoalTagId();
        LLResult latest = robot.outtake.vision.latest;

        telemetry.addData("Controls", "LS/RS=Drive  B=Prime/Execute  X=ResetPose  Y=Cancel/Clear");
        telemetry.addData("Field Centric", useFieldCentric);
        telemetry.addData("Blue Long Start", "x=%.2f y=%.2f h=%.1f",
                blueLongStartPose.getX(),
                blueLongStartPose.getY(),
                Math.toDegrees(blueLongStartPose.getHeading()));

        telemetry.addData("Reloc Primed", robot.relocalization.isGoalTagRelocalizationPrimed());
        telemetry.addData("Reloc Phase", robot.relocalization.getGoalTagRelocalizePhaseName());
        telemetry.addData("Visible Goal Tag", visibleGoalTag);
        telemetry.addData("Turret Cmd (deg)", "%.1f", robot.outtake.turret.getCurrentDegrees());

        if (latest != null) {
            telemetry.addData("LL Valid", latest.isValid());
            telemetry.addData("LL Staleness (ms)", latest.getStaleness());
            telemetry.addData("LL Tag Count", latest.getBotposeTagCount());
        } else {
            telemetry.addData("LL Valid", false);
            telemetry.addData("LL Staleness (ms)", "N/A");
            telemetry.addData("LL Tag Count", "N/A");
        }

        if (lastRelocalizeResult != null) {
            telemetry.addData("Reloc Success", lastRelocalizeResult.success);
            telemetry.addData("Reloc Tag", lastRelocalizeResult.tagId);
            telemetry.addData("Reloc Reason", lastRelocalizeResult.reason);
            addPoseTelemetry("Pose Before", lastRelocalizeResult.followerPoseBefore);
            addPoseTelemetry("Relocalized Pose", lastRelocalizeResult.relocalizedPose);
        } else {
            telemetry.addData("Reloc Success", "N/A");
        }

        addPoseTelemetry("Follower Pose", followerPose);
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
}
