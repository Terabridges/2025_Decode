package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

@Configurable
@TeleOp(name = "GoalTagRelocalizationTest", group = "Test")
public class GoalTagRelocalizationTest extends OpMode {

    public static double startX = 72.0;
    public static double startY = 72.0;
    public static double startHeadingDeg = 0.0;
    public static double forwardTurretDeg = Turret.turretForwardDeg;
    public static boolean relocalizeEveryLoop = false;

    private Robot robot;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Robot.GoalTagRelocalizeResult lastRelocalizeResult;
    private Pose lastRelocalizedPose;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
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
        robot.update();

        if (relocalizeEveryLoop || (currentGamepad1.a && !previousGamepad1.a)) {
            lastRelocalizeResult = robot.relocalizeFromGoalTag(forwardTurretDeg);
            if (lastRelocalizeResult.success && lastRelocalizeResult.relocalizedPose != null) {
                lastRelocalizedPose = copyPose(lastRelocalizeResult.relocalizedPose);
            }
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            if (follower != null) {
                follower.setPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
            }
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            lastRelocalizeResult = null;
            lastRelocalizedPose = null;
        }

        drawCurrentAndHistory();
        addTelemetry();
    }

    private void addTelemetry() {
        Pose followerPose = (follower != null) ? follower.getPose() : null;
        int visibleGoalTag = robot.outtake.vision.getVisibleGoalTagId();

        telemetry.addData("Controls", "A=Relocalize B=ResetPose X=ClearResult");
        telemetry.addData("Relocalize Every Loop", relocalizeEveryLoop);
        telemetry.addData("Turret Forward Cmd (deg)", "%.1f", forwardTurretDeg);
        telemetry.addData("Visible Goal Tag", visibleGoalTag);
        telemetry.addData("Turret Cmd (deg)", "%.1f", robot.outtake.turret.getCurrentDegrees());

        if (lastRelocalizeResult != null) {
            telemetry.addData("Relocalize Success", lastRelocalizeResult.success);
            telemetry.addData("Relocalize Tag", lastRelocalizeResult.tagId);
            telemetry.addData("Relocalize Reason", lastRelocalizeResult.reason);
            addPoseTelemetry("Pose Before", lastRelocalizeResult.followerPoseBefore);
            addPoseTelemetry("Relocalized Pose", lastRelocalizedPose);
        } else {
            telemetry.addData("Relocalize Success", "N/A");
        }

        addPoseTelemetry("Follower Pose", followerPose);
        telemetry.update();
    }

    private void addPoseTelemetry(String label, Pose pose) {
        if (pose == null) {
            telemetry.addData(label, "null");
            return;
        }
        telemetry.addData(label, "x=%.2f y=%.2f h=%.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }
}
