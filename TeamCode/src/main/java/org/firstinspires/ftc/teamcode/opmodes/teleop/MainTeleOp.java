package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.LoopTimeTracker;
import org.firstinspires.ftc.teamcode.config.utility.PoseLoggingUtil;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;
import org.psilynx.psikit.ftc.autolog.PsiKitNoFieldAutoLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@PsiKitAutoLog(rlogPort = 5802)
//@PsiKitFieldAutoLog
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double B_LONG_PRESS_RESET_SEC = 0.6;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double ROBOT_WIDTH_IN = 17.5;
    private static final double ROBOT_LENGTH_IN = 18.0;
    public static boolean enableSectionTimingLogs = true;
    public static double autoOffsetStationarySeconds = 1.0;
    // Match the auto shooting "robot settled" gate.
    public static double autoOffsetMaxRobotSpeedInS = 1.5;
    public static double autoOffsetMaxRobotAngularSpeedDegS = 12.0;
    public static boolean enableFieldAutoLog = true;
    public static double fieldAutoLogPeriodSec = 0.10;
    public static double teleopBlueBankTrimOffsetDeg = 2.0;
    public static double teleopRedBankTrimOffsetDeg = 2.0;

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
    private boolean shootRequestPending = false;
    private boolean pendingShootUsesSorting = false;

    private JoinedTelemetry joinedTelemetry;
    @PsiKitNoFieldAutoLog
    private LoopTimeTracker loopTimeTracker;

    public ElapsedTime telemetryTimer;
    public double telemetryTime;
    private ElapsedTime bHoldTimer;
    private boolean bLongPressHandled = false;
    private GoBildaPinpointDriver pinpoint;

    EdgeDetector getReadyShoot = new EdgeDetector(() -> robot.getReadyShoot());
    EdgeDetector toggleSorting = new EdgeDetector(()-> robot.toggleSorting());
    EdgeDetector nextMotif = new EdgeDetector(()-> GlobalVariables.nextMotif());
    EdgeDetector flashLights = new EdgeDetector(()-> robot.toggleLightsTurret());
    EdgeDetector setSpindexCurrentDeg = new EdgeDetector(()-> robot.intake.spindex.setSpindexDegree(robot.intake.spindex.getAbsolutePos()));

    @Override
    public void init() {
        configureLowOverheadPsiKitLogging();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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
        bHoldTimer = new ElapsedTime();

    }

    private void configureLowOverheadPsiKitLogging() {
        FtcLogTuning.bulkOnlyLogging = true;
        FtcLogTuning.nonBulkReadPeriodSec = 0.10;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;
        FtcLogTuning.pinpointLoggerCallsUpdate = false;
        FtcLogTuning.pinpointReadPeriodSec = .10;
        FtcLogTuning.fieldAutoLogEnabled = enableFieldAutoLog;
        FtcLogTuning.fieldAutoLogPeriodSec = fieldAutoLogPeriodSec;
        FtcLogTuning.fieldAutoLogMaxDepth = 4;
        FtcLogTuning.fieldAutoLogIncludeStaticFields = false;
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
            Alliance alliance = GlobalVariables.isBlueAlliance() ? Alliance.BLUE : Alliance.RED;
            Pose teleopStartPose = new AutoPoses().findStartPose(alliance, Range.LONG_RANGE);
            FollowerManager.initFollower(hardwareMap, teleopStartPose);
        }
        // Consume the auto->teleop handoff flag for this start.
        GlobalVariables.setAutoFollowerValid(false);
        robot.outtake.setAimLockEnabled(true);
        Outtake.defaultTurretAimTrimOffsetDeg = getAllianceBankTrimOffsetDeg();
        Outtake.turretAimTrimOffsetDeg = Outtake.defaultTurretAimTrimOffsetDeg;
        // Raise the hood while flywheel RPM is recovering between fast shots.
        Outtake.enableRpmRecoilComp = true;
        bHoldTimer.reset();
        bLongPressHandled = false;

        shootAllMachine.start();
        sortingShootAllMachine.start();
        robot.intake.spindex.emptyBalls();

        loopTimeTracker.reset();
        telemetryTimer.reset();
    }

    @Override
    public void loop() {
        long tLoopStartNs = System.nanoTime();

        gamepadUpdate();
        long tAfterGamepadNs = System.nanoTime();

        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }
        long tAfterFollowerNs = System.nanoTime();

        updateGp2BackImuRecalibration();
        applyAllianceVisionLockConfig();
        long tAfterAllianceNs = System.nanoTime();

        controlsUpdate();
        long tAfterControlsNs = System.nanoTime();

        robot.update();
        long tAfterRobotNs = System.nanoTime();

        logPsiKitData();
        controlsTelemetryUpdate();
        long tAfterTelemetryNs = System.nanoTime();

        stateMachinesUpdate();
        logStateMachinePsiKitData();
        long tAfterStateMachinesNs = System.nanoTime();

        //drawCurrentAndHistory();
        long tAfterDrawingNs = System.nanoTime();

        loopTimeTracker.sampleLoop();
        long tLoopEndNs = System.nanoTime();

        if (enableSectionTimingLogs) {
            Logger.recordOutput("MainTeleOp/TimingMs/GamepadUpdate", nanosToMillis(tAfterGamepadNs - tLoopStartNs));
            Logger.recordOutput("MainTeleOp/TimingMs/FollowerUpdate", nanosToMillis(tAfterFollowerNs - tAfterGamepadNs));
            Logger.recordOutput("MainTeleOp/TimingMs/AllianceConfig", nanosToMillis(tAfterAllianceNs - tAfterFollowerNs));
            Logger.recordOutput("MainTeleOp/TimingMs/ControlsUpdate", nanosToMillis(tAfterControlsNs - tAfterAllianceNs));
            Logger.recordOutput("MainTeleOp/TimingMs/RobotUpdate", nanosToMillis(tAfterRobotNs - tAfterControlsNs));
            Logger.recordOutput("MainTeleOp/TimingMs/ControlsTelemetry", nanosToMillis(tAfterTelemetryNs - tAfterRobotNs));
            Logger.recordOutput("MainTeleOp/TimingMs/StateMachines", nanosToMillis(tAfterStateMachinesNs - tAfterTelemetryNs));
            Logger.recordOutput("MainTeleOp/TimingMs/DrawField", nanosToMillis(tAfterDrawingNs - tAfterStateMachinesNs));
            Logger.recordOutput("MainTeleOp/TimingMs/LoopTrackerSample", nanosToMillis(tLoopEndNs - tAfterDrawingNs));
            Logger.recordOutput("MainTeleOp/TimingMs/TotalLoop", nanosToMillis(tLoopEndNs - tLoopStartNs));
            Logger.recordOutput("MainTeleOp/Spindex/CommandedDeg", robot.intake.spindex.getCommandedDegree());
            Logger.recordOutput("MainTeleOp/Spindex/AbsoluteDeg", robot.intake.spindex.getAbsolutePos());
            Logger.recordOutput("MainTeleOp/StateMachines/SortedStartBall", robot.sortedStartBall);

        }
    }

    @Override
    public void stop() {
    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
        }

        boolean bButtonHeld = currentGamepad1.b || currentGamepad2.b;
        boolean bButtonJustPressed =
                (currentGamepad1.b && !previousGamepad1.b)
                || (currentGamepad2.b && !previousGamepad2.b);
        boolean bButtonJustReleased =
                !bButtonHeld && (previousGamepad1.b || previousGamepad2.b);

        if (bButtonJustPressed) {
            applyBManualAdjustOnce();
            bHoldTimer.reset();
            bLongPressHandled = false;
        }
        if (bButtonHeld && !bLongPressHandled && bHoldTimer.seconds() >= B_LONG_PRESS_RESET_SEC) {
            Outtake.resetTurretAimVisionOffset();
            bLongPressHandled = true;
        }
        if (bButtonJustReleased) {
            bLongPressHandled = false;
        }
        toggleSorting.update(gamepad1.start);
        nextMotif.update(gamepad2.y);
        flashLights.update(gamepad2.x);
        setSpindexCurrentDeg.update(gamepad1.a);
        updateManualFollowerPoseReset();
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
            //joinedTelemetry.addData("Shoot Pending", shootRequestPending);
//            if (robot != null && robot.outtake != null) {
//                joinedTelemetry.addData("In Launch Zone", robot.outtake.isAnyPartInLaunchZone());
//            }
            //joinedTelemetry.addData("TXLights", robot.txLights);
            joinedTelemetry.addData("Turret Aim Offset (deg)", "%.2f", Outtake.getTotalTurretAimCommandOffsetDeg());
            joinedTelemetry.addData("Auto Vision Bias (deg)", "%.2f", Outtake.turretAimAutoVisionBiasDeg);
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

    private void updateGp2BackImuRecalibration() {
        if (currentGamepad2.back && !previousGamepad2.back) {
            if (pinpoint != null) {
                pinpoint.recalibrateIMU();
            }
        }
    }

    public void stateMachinesUpdate(){
        boolean xPressed = currentGamepad1.x && !previousGamepad1.x;
        if (xPressed) {
            if (shootRequestPending) {
                // Manual override: second press while pending starts shooting immediately.
                if (!pendingShootUsesSorting && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                    robot.initShootAllMachine = true;
                    shootRequestPending = false;
                } else if (pendingShootUsesSorting && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                    robot.initSortedShootAllMachine = true;
                    shootRequestPending = false;
                }
            } else {
                if (!robot.useSorting && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                    shootRequestPending = true;
                    pendingShootUsesSorting = false;
                } else if (robot.useSorting && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                    shootRequestPending = true;
                    pendingShootUsesSorting = true;
                }
            }
        }

        if (shootRequestPending
                && robot != null
                && robot.outtake != null
                && robot.outtake.isAnyPartInLaunchZone()) {
            if (!pendingShootUsesSorting && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                robot.initShootAllMachine = true;
                shootRequestPending = false;
            } else if (pendingShootUsesSorting && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                robot.initSortedShootAllMachine = true;
                shootRequestPending = false;
            }
        }
        shootAllMachine.update();
        sortingShootAllMachine.update();
    }

    private void applyAllianceVisionLockConfig() {
        // Keep alliance tag config in one place to avoid drift between start/loop behavior.
        if (GlobalVariables.isBlueAlliance()) {
            robot.outtake.vision.setRequiredTagId(BLUE_GOAL_TAG_ID);
        } else if (GlobalVariables.isRedAlliance()) {
            robot.outtake.vision.setRequiredTagId(RED_GOAL_TAG_ID);
        }
        Outtake.defaultTurretAimTrimOffsetDeg = getAllianceBankTrimOffsetDeg();
        // Teleop should always be goal-targeted; this prevents stray obelisk targeting.
        if (robot.outtake.getAimTarget() != Outtake.AimTarget.GOAL) {
            robot.outtake.setAimTargetGoal();
        }
    }

    private double getAllianceBankTrimOffsetDeg() {
        return GlobalVariables.isRedAlliance() ? teleopRedBankTrimOffsetDeg : teleopBlueBankTrimOffsetDeg;
    }

    private void logPsiKitData() {
        PoseLoggingUtil.logMainPoseDetails(robot);
    }

    private void applyBManualAdjustOnce() {
        if (robot == null || robot.outtake == null || robot.outtake.vision == null) {
            return;
        }
        int requiredTagId = robot.outtake.vision.getRequiredTagId();
        if (!robot.outtake.vision.hasRequiredTarget()) {
            return;
        }
        Outtake.commitAutoVisionBiasAndTxToManualOffset(robot.outtake.vision.getTxForTag(requiredTagId));
    }

    private void updateManualFollowerPoseReset() {
        if (FollowerManager.follower == null) {
            return;
        }

        boolean gp2LeftPressed = currentGamepad2.dpad_left && !previousGamepad2.dpad_left;
        boolean gp2RightPressed = currentGamepad2.dpad_right && !previousGamepad2.dpad_right;

        if (!gp2LeftPressed && !gp2RightPressed) {
            return;
        }
        Outtake.resetTurretAimOffsets();

        Pose resetPose;
        if (gp2LeftPressed) {
            if (GlobalVariables.isBlueAlliance()) {
                resetPose = new Pose(
                        FIELD_SIZE_IN - (ROBOT_LENGTH_IN / 2.0),
                        ROBOT_WIDTH_IN / 2.0,
                        Math.toRadians(0.0)
                );
            } else {
                resetPose = new Pose(
                        ROBOT_LENGTH_IN / 2.0,
                        ROBOT_WIDTH_IN / 2.0,
                        Math.toRadians(0.0)
                );
            }
        } else {
            if (GlobalVariables.isBlueAlliance()) {
                resetPose = new Pose(
                        48.0 + (ROBOT_LENGTH_IN / 2.0),
                        FIELD_SIZE_IN - (ROBOT_WIDTH_IN / 2.0),
                        Math.toRadians(180.0)
                );
            } else {
                resetPose = new Pose(
                        FIELD_SIZE_IN - 48.0 - (ROBOT_LENGTH_IN / 2.0),
                        FIELD_SIZE_IN - (ROBOT_WIDTH_IN / 2.0),
                        Math.toRadians(180.0)
                );
            }
        }

        FollowerManager.follower.setPose(resetPose);
    }
    private void logStateMachinePsiKitData() {
        double spindexCommandedDeg = robot.intake.spindex.getCommandedDegree();
        double spindexAbsoluteDeg = robot.intake.spindex.getAbsolutePos();
        double shooterTargetRpm = robot.outtake.shooter.getTargetRPM();
        double shooterCurrentRpm = robot.outtake.shooter.getCurrentRPM();
        boolean spindexAtPos = robot.intake.spindex.isSpindexAtPos();
        boolean shooterAtRpm = robot.outtake.shooter.isAtRPM();
        boolean unJamRequested = robot.other.unJam;
        boolean goToResetPending = robot.isGoToResetPending();

        if (shootAllMachine != null) {
            Object shootState = shootAllMachine.getState();
            Logger.recordOutput("MainTeleOp/StateMachines/ShootAll/State", String.valueOf(shootState));
            Logger.recordOutput(
                    "MainTeleOp/StateMachines/ShootAll/InInit",
                    Robot.ShootAllStates.INIT.equals(shootState)
            );
            logShootAllTransitionInputs((Robot.ShootAllStates) shootState, spindexAtPos, shooterAtRpm, unJamRequested, goToResetPending);
        }

        if (sortingShootAllMachine != null) {
            Object sortedState = sortingShootAllMachine.getState();
            Logger.recordOutput("MainTeleOp/StateMachines/SortedShootAll/State", String.valueOf(sortedState));
            Logger.recordOutput(
                    "MainTeleOp/StateMachines/SortedShootAll/InInit",
                    Robot.SortedShootAllStates.INIT.equals(sortedState)
            );
            logSortedShootAllTransitionInputs((Robot.SortedShootAllStates) sortedState, spindexAtPos, shooterAtRpm, unJamRequested, goToResetPending);
        }

        Logger.recordOutput("MainTeleOp/StateMachines/ShootRequestPending", shootRequestPending);
        Logger.recordOutput("MainTeleOp/StateMachines/PendingShootUsesSorting", pendingShootUsesSorting);
        Logger.recordOutput("MainTeleOp/StateMachines/InitShootAllMachine", robot.initShootAllMachine);
        Logger.recordOutput("MainTeleOp/StateMachines/InitSortedShootAllMachine", robot.initSortedShootAllMachine);
        Logger.recordOutput("MainTeleOp/StateMachines/UseSorting", robot.useSorting);
        Logger.recordOutput("MainTeleOp/StateMachines/SortedStartBall", robot.sortedStartBall);
        Logger.recordOutput("MainTeleOp/StateMachines/LoadedBallCount", robot.getLoadedBallCount());
        Logger.recordOutput("MainTeleOp/StateMachines/Spindex/CommandedDeg", spindexCommandedDeg);
        Logger.recordOutput("MainTeleOp/StateMachines/Spindex/AbsoluteDeg", spindexAbsoluteDeg);
        Logger.recordOutput("MainTeleOp/StateMachines/Spindex/AtPos", spindexAtPos);
        Logger.recordOutput("MainTeleOp/StateMachines/Shooter/TargetRpm", shooterTargetRpm);
        Logger.recordOutput("MainTeleOp/StateMachines/Shooter/CurrentRpm", shooterCurrentRpm);
        Logger.recordOutput("MainTeleOp/StateMachines/Shooter/AtRpm", shooterAtRpm);
        Logger.recordOutput("MainTeleOp/StateMachines/Shooter/RpmError", shooterTargetRpm - shooterCurrentRpm);
        Logger.recordOutput("MainTeleOp/StateMachines/UnJamRequested", unJamRequested);
        Logger.recordOutput("MainTeleOp/StateMachines/GoToResetPending", goToResetPending);
        Logger.recordOutput("MainTeleOp/StateMachines/WaitTimeSec", robot.getShootAllWaitTime());
    }

    private void logShootAllTransitionInputs(
            Robot.ShootAllStates state,
            boolean spindexAtPos,
            boolean shooterAtRpm,
            boolean unJamRequested,
            boolean goToResetPending
    ) {
        String prefix = "MainTeleOp/StateMachines/ShootAll/Next";
        switch (state) {
            case INIT:
                Logger.recordOutput(prefix + "/InitRequested", robot.initShootAllMachine);
                break;
            case GO_TO_SHOOT_ONE:
                Logger.recordOutput(prefix + "/SpindexAtPos", spindexAtPos);
                Logger.recordOutput(prefix + "/ShooterAtRpm", shooterAtRpm);
                Logger.recordOutput(prefix + "/AdvanceReady", spindexAtPos && shooterAtRpm);
                Logger.recordOutput(prefix + "/UnJamRequested", unJamRequested);
                break;
            case WAIT0:
            case GO_TO_SHOOT_TWO:
            case GO_TO_SHOOT_THREE:
            case RESET:
                Logger.recordOutput(prefix + "/SpindexAtPos", spindexAtPos);
                Logger.recordOutput(prefix + "/UnJamRequested", unJamRequested);
                break;
            case WAIT1:
            case WAIT2:
            case WAIT3:
                Logger.recordOutput(prefix + "/WaitTimeSec", robot.getShootAllWaitTime());
                Logger.recordOutput(prefix + "/UnJamRequested", unJamRequested);
                break;
            case UNJAM:
                Logger.recordOutput(prefix + "/GoToResetPending", goToResetPending);
                break;
        }
    }

     private void logSortedShootAllTransitionInputs(
            Robot.SortedShootAllStates state,
            boolean spindexAtPos,
            boolean shooterAtRpm,
            boolean unJamRequested,
            boolean goToResetPending
    ) {
        String prefix = "MainTeleOp/StateMachines/SortedShootAll/Next";
        switch (state) {
            case INIT:
                Logger.recordOutput(prefix + "/InitRequested", robot.initSortedShootAllMachine);
                Logger.recordOutput(prefix + "/SelectedStartBall", robot.sortedStartBall);
                break;
            case GO_TO_FIRST:
                Logger.recordOutput(prefix + "/SpindexAtPos", spindexAtPos);
                Logger.recordOutput(prefix + "/ShooterAtRpm", shooterAtRpm);
                Logger.recordOutput(prefix + "/AdvanceReady", spindexAtPos && shooterAtRpm);
                Logger.recordOutput(prefix + "/UnJamRequested", unJamRequested);
                Logger.recordOutput(prefix + "/SelectedStartBall", robot.sortedStartBall);
                break;
            case WAIT0:
            case GO_TO_SECOND:
            case GO_TO_THIRD:
            case RESET:
                Logger.recordOutput(prefix + "/SpindexAtPos", spindexAtPos);
                Logger.recordOutput(prefix + "/UnJamRequested", unJamRequested);
                Logger.recordOutput(prefix + "/SelectedStartBall", robot.sortedStartBall);
                break;
            case WAIT1:
            case WAIT2:
            case WAIT3:
                Logger.recordOutput(prefix + "/WaitTimeSec", robot.getShootAllWaitTime());
                Logger.recordOutput(prefix + "/UnJamRequested", unJamRequested);
                Logger.recordOutput(prefix + "/SelectedStartBall", robot.sortedStartBall);
                break;
            case UNJAM:
                Logger.recordOutput(prefix + "/GoToResetPending", goToResetPending);
                Logger.recordOutput(prefix + "/SelectedStartBall", robot.sortedStartBall);
                break;
        }
    }

    private static double nanosToMillis(long nanos) {
        return nanos / 1_000_000.0;
    }
}

