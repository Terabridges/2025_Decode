package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretAimController;

@Configurable
@TeleOp(name = "LimelightPoseCenteringTest", group = "Test")
public class LimelightPoseCenteringTest extends OpMode {

    public static int blueTagId = 20;
    public static int redTagId = 24;
    public static double centerX = 72.0;
    public static double centerY = 72.0;
    public static double turretScanMinDeg = 40.0;
    public static double turretScanMaxDeg = 320.0;
    public static double turretScanPeriodSec = 3.0;
    public static double chassisTurnKp = 0.02;
    public static double chassisTurnMax = 0.30;
    public static double chassisTxDeadbandDeg = 1.5;
    public static double chassisTurnSign = 1.0;
    public static int alignedLoopsRequired = 8;
    public static double turretForwardToleranceDeg = 2.0;

    private enum State {
        SEARCH_TAG,
        ALIGN_CHASSIS_TO_TAG,
        CENTER_TURRET,
        SEED_POSE_FROM_LIMELIGHT,
        DRIVE_TO_CENTER,
        DONE
    }

    private Robot robot;
    private State state = State.SEARCH_TAG;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime scanTimer = new ElapsedTime();
    private int activeTagId = -1;
    private int alignedLoops = 0;
    private PathChain toCenterPath;
    private boolean centerPathStarted = false;
    private boolean limelightPoseApplied = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        robot.outtake.vision.toInit();
        robot.outtake.turret.toInit();
        robot.other.drive.toInit();

        robot.outtake.turret.setAimLockEnabled(false);

        FollowerManager.initFollower(hardwareMap, new Pose(72, 72, 0));

        enterState(State.SEARCH_TAG);
    }

    @Override
    public void loop() {
        robot.outtake.vision.update();
        robot.outtake.turret.update();
        if (follower != null) {
            follower.update();
        }

        switch (state) {
            case SEARCH_TAG:
                doSearchTag();
                break;
            case ALIGN_CHASSIS_TO_TAG:
                doAlignChassisToTag();
                break;
            case CENTER_TURRET:
                doCenterTurret();
                break;
            case SEED_POSE_FROM_LIMELIGHT:
                doSeedPoseFromLimelight();
                break;
            case DRIVE_TO_CENTER:
                doDriveToCenter();
                break;
            case DONE:
                stopChassis();
                break;
        }

        drawCurrentAndHistory();
        addTelemetry();
    }

    private void doSearchTag() {
        stopChassis();

        double t = Math.max(0.0001, turretScanPeriodSec);
        double phase = (scanTimer.seconds() / t) * (2.0 * Math.PI);
        double scanMid = (turretScanMinDeg + turretScanMaxDeg) * 0.5;
        double scanAmp = (turretScanMaxDeg - turretScanMinDeg) * 0.5;
        double targetTurretDeg = scanMid + scanAmp * Math.sin(phase);
        robot.outtake.turret.setTargetAngle(targetTurretDeg);

        if (robot.outtake.vision.seesTag(blueTagId)) {
            activeTagId = blueTagId;
            enterState(State.ALIGN_CHASSIS_TO_TAG);
        } else if (robot.outtake.vision.seesTag(redTagId)) {
            activeTagId = redTagId;
            enterState(State.ALIGN_CHASSIS_TO_TAG);
        }
    }

    private void doAlignChassisToTag() {
        if (activeTagId < 0 || !robot.outtake.vision.seesTag(activeTagId)) {
            enterState(State.SEARCH_TAG);
            return;
        }

        double tx = robot.outtake.vision.getTxForTag(activeTagId);
        double turn = clamp(tx * chassisTurnKp * chassisTurnSign, -chassisTurnMax, chassisTurnMax);
        if (Math.abs(tx) <= chassisTxDeadbandDeg) {
            alignedLoops++;
            turn = 0.0;
        } else {
            alignedLoops = 0;
        }

        driveRotate(turn);

        if (alignedLoops >= Math.max(1, alignedLoopsRequired)) {
            enterState(State.CENTER_TURRET);
        }
    }

    private void doCenterTurret() {
        stopChassis();
        robot.outtake.turret.setTargetAngle(TurretAimController.turretForwardDeg);
        double err = wrapSignedDeg(TurretAimController.turretForwardDeg - robot.outtake.turret.getCurrentDegrees());
        if (Math.abs(err) <= turretForwardToleranceDeg) {
            enterState(State.SEED_POSE_FROM_LIMELIGHT);
        }
    }

    private void doSeedPoseFromLimelight() {
        stopChassis();

        Pose3D llPose = getLimelightBotPose();
        if (llPose != null) {
            // Limelight botpose is reported in meters; convert to inches for Pedro.
            double xIn = llPose.getPosition().x * 39.3701;
            double yIn = llPose.getPosition().y * 39.3701;
            double headingRad = Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES));
            if (follower != null) {
                follower.setPose(new Pose(xIn, yIn, headingRad));
                limelightPoseApplied = true;
            }
            enterState(State.DRIVE_TO_CENTER);
            return;
        }

        // If no pose yet, keep waiting briefly before returning to search.
        if (stateTimer.seconds() > 1.0) {
            enterState(State.SEARCH_TAG);
        }
    }

    private void doDriveToCenter() {
        stopChassis();

        if (follower == null) {
            enterState(State.DONE);
            return;
        }

        if (!centerPathStarted) {
            Pose current = follower.getPose();
            if (current == null) {
                enterState(State.DONE);
                return;
            }
            Pose center = new Pose(centerX, centerY, current.getHeading());
            toCenterPath = follower.pathBuilder()
                    .addPath(new BezierLine(current, center))
                    .setLinearHeadingInterpolation(current.getHeading(), center.getHeading())
                    .build();
            if (toCenterPath != null) {
                follower.followPath(toCenterPath, true);
                centerPathStarted = true;
            } else {
                enterState(State.DONE);
            }
            return;
        }

        if (!follower.isBusy()) {
            enterState(State.DONE);
        }
    }

    private Pose3D getLimelightBotPose() {
        LLResult latest = robot.outtake.vision.latest;
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            Pose3D mt2 = latest.getBotpose_MT2();
            if (mt2 != null) {
                return mt2;
            }
        } catch (Throwable ignored) {
        }
        try {
            return latest.getBotpose();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private void enterState(State next) {
        state = next;
        stateTimer.reset();
        if (next == State.SEARCH_TAG) {
            scanTimer.reset();
            activeTagId = -1;
            alignedLoops = 0;
            centerPathStarted = false;
            toCenterPath = null;
        }
    }

    private void driveRotate(double turn) {
        robot.other.drive.manualDrive = true;
        robot.other.drive.drive(0.0, 0.0, turn);
        robot.other.drive.update();
    }

    private void stopChassis() {
        robot.other.drive.manualDrive = true;
        robot.other.drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
        robot.other.drive.update();
    }

    private static double wrapSignedDeg(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void addTelemetry() {
        Pose p = (follower != null) ? follower.getPose() : null;
        telemetry.addData("State", state);
        telemetry.addData("Tag Seen", activeTagId);
        telemetry.addData("LL hasTarget", robot.outtake.vision.hasTarget());
        telemetry.addData("LL Tx(tag)", activeTagId >= 0 ? robot.outtake.vision.getTxForTag(activeTagId) : 0.0);
        telemetry.addData("Turret Deg", "%.1f", robot.outtake.turret.getCurrentDegrees());
        telemetry.addData("AlignedLoops", alignedLoops);
        telemetry.addData("LL Pose Applied", limelightPoseApplied);
        if (p != null) {
            telemetry.addData("Follower X", "%.2f", p.getX());
            telemetry.addData("Follower Y", "%.2f", p.getY());
            telemetry.addData("Follower H", "%.1f", Math.toDegrees(p.getHeading()));
        }
        telemetry.update();
    }
}
