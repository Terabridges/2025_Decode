package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoTurretAim;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

@Configurable
@TeleOp(name = "AutoAimParityTest", group = "Test")
public class AutoAimParityTest extends OpMode {
    public static boolean startBlueAlliance = true;
    public static boolean startInAcquireMotif = false;
    public static double startX = 57.0;
    public static double startY = 8.75;
    public static double startHeadingBlueDeg = 0.0;
    public static double startHeadingRedDeg = 0.0;
    public static boolean enableManualDrive = true;
    public static boolean preloadComplete = false;

    private Robot robot;
    private AutoTurretAim turretAim;
    private Alliance alliance;
    private AutoStates activeState;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        robot.toInit();
        robot.other.drive.manualDrive = true;

        applyAlliance(startBlueAlliance ? Alliance.BLUE : Alliance.RED);
        activeState = startInAcquireMotif ? AutoStates.ACQUIRE_MOTIF : AutoStates.GO_TO_SHOOT;

        double startHeadingDeg = (alliance == Alliance.BLUE) ? startHeadingBlueDeg : startHeadingRedDeg;
        FollowerManager.initFollower(hardwareMap, new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.x && !previousGamepad1.x) {
            applyAlliance(Alliance.BLUE);
        }
        if (currentGamepad1.y && !previousGamepad1.y) {
            applyAlliance(Alliance.RED);
        }
        if (currentGamepad1.a && !previousGamepad1.a) {
            activeState = (activeState == AutoStates.ACQUIRE_MOTIF)
                    ? AutoStates.GO_TO_SHOOT
                    : AutoStates.ACQUIRE_MOTIF;
        }

        if (follower != null) {
            follower.update();
        }
        setDriveFromGamepad();

        // Keep loop ordering identical to BaseAuto for parity diagnostics.
        turretAim.updateAim(activeState, preloadComplete);
        robot.update();

        drawCurrentAndHistory();
        addTelemetry();
    }

    private void setDriveFromGamepad() {
        if (!enableManualDrive) {
            robot.other.drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
            return;
        }

        double axial = -currentGamepad1.left_stick_y;
        double lateral = currentGamepad1.left_stick_x;
        double yaw = currentGamepad1.right_stick_x;
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

    private void addTelemetry() {
        Pose pose = (follower != null) ? follower.getPose() : null;
        int requiredTag = robot.outtake.vision.getRequiredTagId();
        boolean seesRequired = requiredTag >= 0 && robot.outtake.vision.seesTag(requiredTag);
        double txRequired = seesRequired ? robot.outtake.vision.getTxForTag(requiredTag) : 0.0;
        double distRequired = seesRequired ? robot.outtake.vision.getDistanceInchesForTag(requiredTag) : 0.0;
        double odoGoalDesired = robot.outtake.turret.getOdoGoalDesiredHeadingDeg(robot.outtake.vision);
        double odoGoalDelta = robot.outtake.turret.getOdoGoalHeadingDeltaDeg(robot.outtake.vision);

        telemetry.addData("State", activeState);
        telemetry.addData("Alliance (X blue / Y red)", alliance);
        telemetry.addData("Aim Lock", robot.outtake.turret.isAimLockEnabled());
        telemetry.addData("Aim Source", robot.outtake.turret.getActiveLockSource());
        telemetry.addData("Required Tag", requiredTag);
        telemetry.addData("Sees Required", seesRequired);
        telemetry.addData("Current Seen Tag", robot.outtake.vision.getCurrentTagId());
        telemetry.addData("Tx Required (deg)", txRequired);
        telemetry.addData("Dist Required (in)", distRequired);
        telemetry.addData("Turret Deg", "%.2f", robot.outtake.turret.getCurrentDegrees());
        if (Double.isNaN(odoGoalDesired) || Double.isNaN(odoGoalDelta)) {
            telemetry.addData("ODO Goal Desired (deg)", "N/A");
            telemetry.addData("ODO Goal Delta (deg)", "N/A");
        } else {
            telemetry.addData("ODO Goal Desired (deg)", "%.2f", odoGoalDesired);
            telemetry.addData("ODO Goal Delta (deg)", "%.2f", odoGoalDelta);
        }
        if (pose != null) {
            telemetry.addData("Follower Pose", "x=%.2f y=%.2f h=%.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        } else {
            telemetry.addData("Follower Pose", "N/A");
        }
        telemetry.addData("Controls", "A toggle ACQUIRE/GO_TO_SHOOT, X blue, Y red");
        telemetry.update();
    }

    private void applyAlliance(Alliance nextAlliance) {
        alliance = nextAlliance;
        if (alliance == Alliance.BLUE) {
            GlobalVariables.setAllianceColor(GlobalVariables.AllianceColor.BLUE);
        } else {
            GlobalVariables.setAllianceColor(GlobalVariables.AllianceColor.RED);
        }
        turretAim = new AutoTurretAim(robot, new AutoPoses(), alliance, Range.CLOSE_RANGE, telemetry);
    }
}
