package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.control.Intake.ClutchControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpindexControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpinnerControl;
import org.firstinspires.ftc.teamcode.config.control.Other.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.Other.LiftControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.LightsControl;
import org.firstinspires.ftc.teamcode.config.control.Other.OtherControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.OuttakeControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.TurretControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.VisionControl;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.LoopTimeTracker;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double BLUE_VISION_DIRECTION = -1.0;
    private static final double RED_VISION_DIRECTION = -1.0;
    private static final double SHOOT_HOLD_CANCEL_STICK_THRESHOLD = 0.5;

    IntakeControl intakeControl;
    OuttakeControl outtakeControl;
    OtherControl otherControl;
    ClutchControl clutchControl;
    SpindexControl spindexControl;
    SpinnerControl spinnderControl;
    LiftControl liftControl;
    LightsControl lightsControl;
    DriveControl driveControl;
    ShooterControl shooterControl;
    TurretControl turretControl;
    VisionControl visionControl;

    List<Control> controls;
    Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    StateMachine shootAllMachine;
    StateMachine sortingShootAllMachine;

    private JoinedTelemetry joinedTelemetry;
    private boolean shooterPositionHoldEnabled = true;
    private boolean shooterPositionHoldActive = false;
    private boolean shooterPositionHoldCanceledByDriver = false;
    private Pose shooterHoldPose = null;

    private LoopTimeTracker loopTimeTracker;

    public ElapsedTime telemetryTimer;
    public double telemetryTime;

    EdgeDetector getReadyShoot = new EdgeDetector(() -> robot.getReadyShoot());
    EdgeDetector toggleSorting = new EdgeDetector(()-> robot.toggleSorting());
    EdgeDetector nextMotif = new EdgeDetector(()-> GlobalVariables.nextMotif());
    EdgeDetector flashLights = new EdgeDetector(()-> robot.toggleLightsTurret());
    EdgeDetector toggleShooterPositionHold = new EdgeDetector(() -> shooterPositionHoldEnabled = !shooterPositionHoldEnabled);
    EdgeDetector relocalizeDoubleB = new EdgeDetector(() -> robot.outtake.relocalization.onRelocalizeButtonPress());

    @Override
    public void init() {
        configureLowOverheadPsiKitLogging();

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
        outtakeControl = new OuttakeControl(robot, gamepad1, gamepad2);
        otherControl = new OtherControl(robot, gamepad1, gamepad2);
        clutchControl = new ClutchControl(robot, gamepad1, gamepad2);
        spindexControl = new SpindexControl(robot, gamepad1, gamepad2);
        spinnderControl = new SpinnerControl(robot, gamepad1, gamepad2);
        liftControl = new LiftControl(robot, gamepad1, gamepad2);
        lightsControl = new LightsControl(robot, gamepad1, gamepad2);
        driveControl = new DriveControl(robot, gamepad1, gamepad2);
        shooterControl = new ShooterControl(robot, gamepad1, gamepad2);
        turretControl = new TurretControl(robot, gamepad1, gamepad2);
        visionControl = new VisionControl(robot, gamepad1, gamepad2);


        controls = new ArrayList<>(Arrays.asList(intakeControl, outtakeControl, otherControl, clutchControl, spindexControl, spinnderControl, liftControl, lightsControl, driveControl, shooterControl, turretControl, visionControl));

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        shootAllMachine = robot.getShootAllMachine();
        sortingShootAllMachine = robot.getSortedShootAllMachine();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        loopTimeTracker = new LoopTimeTracker();
        telemetryTimer = new ElapsedTime();

    }

    private void configureLowOverheadPsiKitLogging() {
        FtcLogTuning.nonBulkReadPeriodSec = 0.10;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;
        FtcLogTuning.pinpointLoggerCallsUpdate = false;
        FtcLogTuning.pinpointReadPeriodSec = .10;
    }

    @Override
    public void init_loop(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.a && !previousGamepad1.a){
            GlobalVariables.nextMotif();
        }

        if (currentGamepad1.b && !previousGamepad1.b){
            GlobalVariables.toggleAlliance();
        }

        telemetry.addData("Press A to change Motif. Press B to change alliance color.", "");
        telemetry.addData("Motif", GlobalVariables.getMotif());
        telemetry.addData("Alliance Color", GlobalVariables.getAllianceColor());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.toInit();
        applyAllianceVisionLockConfig();
        boolean reuseAutoFollower = GlobalVariables.isAutoFollowerValid()
                && FollowerManager.follower != null;
        if (reuseAutoFollower) {
            FollowerManager.getFollower(hardwareMap);
        } else {
            double allianceHeading = GlobalVariables.isBlueAlliance() ? Math.PI : 0.0;
            FollowerManager.initFollower(hardwareMap, new Pose(72, 72, allianceHeading));
        }
        // Consume the auto->teleop handoff flag for this start.
        GlobalVariables.setAutoFollowerValid(false);
        robot.outtake.turret.setAimLockEnabled(true);

        shootAllMachine.start();
        sortingShootAllMachine.start();

        loopTimeTracker.reset();
        telemetryTimer.reset();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }
        updateAllianceToggle();
        applyAllianceVisionLockConfig();
        controlsUpdate();
        robot.update();
        controlsTelemetryUpdate();
        stateMachinesUpdate();
        updateShooterPositionHold();
        drawCurrentAndHistory();
        loopTimeTracker.sampleLoop();
    }

    @Override
    public void stop() {
    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
        }
        getReadyShoot.update(gamepad2.b);
        toggleSorting.update(gamepad1.start || gamepad2.start);
        nextMotif.update(gamepad2.y);
        flashLights.update(gamepad2.right_bumper);
        toggleShooterPositionHold.update(gamepad2.left_bumper);
        relocalizeDoubleB.update(gamepad1.b);
    }

    public void controlsTelemetryUpdate() {
        if (telemetryTimer.milliseconds()>200) {
            for (Control c : controls) {
                c.addTelemetry(joinedTelemetry);
            }
            if (FollowerManager.follower != null && FollowerManager.follower.getPose() != null) {
                Pose pose = FollowerManager.follower.getPose();
//                joinedTelemetry.addData("Pedro X", "%.2f", pose.getX());
//                joinedTelemetry.addData("Pedro Y", "%.2f", pose.getY());
//                joinedTelemetry.addData("Pedro H (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
            }
            joinedTelemetry.addData("Alliance", GlobalVariables.getAllianceColorName());
            joinedTelemetry.addData("Motif", GlobalVariables.getMotif());
                joinedTelemetry.addData(
                    "Loop (ms)",
                    "now %.2f | avg %.2f",
                    loopTimeTracker.getCurrentLoopTimeMs(),
                    loopTimeTracker.getTrailingAverageMs()
                );
            joinedTelemetry.addData("Use Sorting", robot.useSorting);
            joinedTelemetry.addData("TXLights", robot.txLights);
            joinedTelemetry.addData("Reloc State", robot.outtake.relocalization.getState());
            joinedTelemetry.addData("Reloc Last Success", robot.outtake.relocalization.wasLastSuccess());
            joinedTelemetry.addData("Reloc Reason", robot.outtake.relocalization.getLastReason());
            joinedTelemetry.addData("Reloc Tag Visible (prep)", robot.outtake.relocalization.isPreparedTagVisible());
            joinedTelemetry.update();

            telemetryTimer.reset();
        }
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    private void updateAllianceToggle() {
        if (currentGamepad2.back && !previousGamepad2.back) {
            GlobalVariables.toggleAlliance();
        }
    }

    public void stateMachinesUpdate(){
        if(!robot.useSorting) {
            if (currentGamepad1.x && !previousGamepad1.x && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                robot.initShootAllMachine = true;
            }
        } else {
            if (currentGamepad1.x && !previousGamepad1.x && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                robot.initSortedShootAllMachine = true;
            }
        }
        shootAllMachine.update();
        sortingShootAllMachine.update();
    }

    private void updateShooterPositionHold() {
        if (!shooterPositionHoldEnabled) {
            releaseShooterPositionHold();
            shooterPositionHoldCanceledByDriver = false;
            return;
        }

        boolean shouldHold = isAnyShootMachineActive();
        if (!shouldHold) {
            releaseShooterPositionHold();
            shooterPositionHoldCanceledByDriver = false;
            return;
        }

        if (FollowerManager.follower == null) {
            releaseShooterPositionHold();
            return;
        }

        if (isDriverRequestingHoldCancel()) {
            shooterPositionHoldCanceledByDriver = true;
            releaseShooterPositionHold();
            return;
        }

        if (shooterPositionHoldCanceledByDriver) {
            robot.other.drive.manualDrive = true;
            return;
        }

        if (!shooterPositionHoldActive) {
            Pose currentPose = FollowerManager.follower.getPose();
            if (currentPose == null) {
                return;
            }
            shooterHoldPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
            shooterPositionHoldActive = true;
        }

        robot.other.drive.manualDrive = false;
        robot.other.drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
        FollowerManager.follower.holdPoint(shooterHoldPose);
    }

    private boolean isAnyShootMachineActive() {
        if (robot == null) {
            return false;
        }

        StateMachine activeShootMachine = robot.useSorting ? sortingShootAllMachine : shootAllMachine;
        if (activeShootMachine == null) {
            return false;
        }

        Object state = activeShootMachine.getState();
        if (state == null) {
            return false;
        }

        // Sorted shoot machine can currently emit either enum type for INIT.
        return !state.equals(Robot.SortedShootAllStates.INIT)
                && !state.equals(Robot.ShootAllStates.INIT);
    }

    private void releaseShooterPositionHold() {
        if (!shooterPositionHoldActive) {
            return;
        }

        if (FollowerManager.follower != null) {
            FollowerManager.follower.breakFollowing();
        }
        shooterPositionHoldActive = false;
        shooterHoldPose = null;
        robot.other.drive.manualDrive = true;
        robot.other.drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
    }

    private boolean isDriverRequestingHoldCancel() {
        return Math.abs(gamepad1.left_stick_x) >= SHOOT_HOLD_CANCEL_STICK_THRESHOLD
                || Math.abs(gamepad1.left_stick_y) >= SHOOT_HOLD_CANCEL_STICK_THRESHOLD
                || Math.abs(gamepad1.right_stick_x) >= SHOOT_HOLD_CANCEL_STICK_THRESHOLD;
    }

    private void applyAllianceVisionLockConfig() {
        // Keep alliance lock config in one place to avoid drift between start/loop behavior.
        Turret.cameraLateralOffsetIn = 0.0;
        if (GlobalVariables.isBlueAlliance()) {
            robot.outtake.vision.setRequiredTagId(BLUE_GOAL_TAG_ID);
            Turret.visionDirection = BLUE_VISION_DIRECTION;
        } else if (GlobalVariables.isRedAlliance()) {
            robot.outtake.vision.setRequiredTagId(RED_GOAL_TAG_ID);
            Turret.visionDirection = RED_VISION_DIRECTION;
        }
    }
}
