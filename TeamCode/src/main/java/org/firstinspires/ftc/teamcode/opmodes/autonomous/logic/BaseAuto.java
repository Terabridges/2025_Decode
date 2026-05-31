package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;


import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrent;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.telemetryM;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoIntakeSpeed;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoMotifTracker;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPathLibrary;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoRoutePlanner;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoTurretAim;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

import org.firstinspires.ftc.teamcode.config.utility.PoseLoggingUtil;

public abstract class BaseAuto extends OpMode {

    // ===== Pathing Configuration =====
    private final AutoPoses poses = new AutoPoses();
    private final AutoPathLibrary pathLibrary = new AutoPathLibrary(poses);
    private Pose startPose;
    private PathChain goToPickupPath;
    private PathChain pickupPath;
    private PathChain goToScorePath;
    private PathChain backRowLoopPickupPath;
    private PathChain backRowLoopCompletePickupPath;
    private PathChain leavePath;
    private PathChain releaseCompletePath;
    private double intakeSpeed = 0.40;

//    private final AutoIntakeSpeed intakeSpeedModel = new AutoIntakeSpeed(
//            -0.02, 0.47, 0.18, 0.24, -0.01, 0.01);

    // ===== Constants =====
    private static final double SHOOT_ACTION_SECONDS = 2.5;
    private static final double COMPLETE_SHOOT_TURRET_TOLERANCE_DEG = 2.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 1.5;
    private static final double MOTIF_ACQUIRE_AIM_WINDOW_SECONDS = 0.5;
    private static final double STATE_TIMEOUT_SECONDS = 4.0; // fallback: force state advance after this time
    private static final double GO_TO_PICKUP_IDLE_HOLD_SECONDS = 0.0;
    private static final double PATH_ADVANCE_PROGRESS = 0.90;
    private static final double GO_TO_PICKUP_PATH_ADVANCE_PROGRESS = 0.75;
    private static final double ROW4_PICKUP_TIMEOUT_SECONDS = 1.75;
    private static final double ROW1_PICKUP_TIMEOUT_SECONDS = 2.5;
    private static final double BACKROW_PICKUP_TIMEOUT_SECONDS = 2.5;
    private static final double FAR_PICKUP_ZONE_POWER = 1.0;
    private static final double CLOSE_LOOP_PICKUP_ZONE_POWER = 1.0;
    private static final double CLOSE_LOOP_PICKUP_PART2_POWER = 1.0;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_FIRST_HALF_POWER = 1.0;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_SLOWDOWN_START_T = 0.50;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_SECOND_HALF_POWER = 1.0;
    private static final double BACKROW_COMPLETE_PICKUP_POWER = 1.0;
    private static final double ROW4_COMPLETE_PICKUP_POWER = 1.0;
    private static final double ROW3_COMPLETE_PICKUP_POWER = 0.40;
    private static final double CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS = 1.05;
    private static final double CLOSE_LOOP_GO_TO_PICKUP_IDLE_DELAY_SECONDS = 0.0;
    private static final double FAR_BACKROW_GO_TO_PICKUP_IDLE_HOLD_SECONDS = 0.0;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_TIMEOUT_SECONDS = 2.0;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_IDLE_DELAY_SECONDS = 0.0;
    private static final double PICKUP_HEADING_TOLERANCE_DEG = 3.0;
    private static final int PICKUP_TARGET_BALL_COUNT = 3;
    private static final int TAG_BLUE = 20;
    private static final int TAG_RED = 24;
    private static final double RELEASE_IDLE_SECONDS = 0.0;
    private static final double RELEASE_TIMEOUT_SECONDS = 1.5;
    private static final double RELEASE_COMPLETE_POWER = 1.0;
    private static final boolean SHOOT_WHILE_MOVING_ENABLED = true;
    private static final double RED_CLOSE_PRELOAD_GO_TO_SHOOT_POWER = 1.0;
    private static final double GO_TO_SHOOT_PATH_ADVANCE_PROGRESS = 0.85;
    private static final double SHOOT_START_PATH_PROGRESS = 0.85;
    private static final double READY_SHOOT_FULL_PATH_PROGRESS = 0.20;
    private static final double READY_SHOOT_PARTIAL_PATH_PROGRESS = 0.80;
    private static final double OUTTAKE_REVERSE_PULSE_START_PATH_PROGRESS = 0.15;
    private static final double OUTTAKE_REVERSE_PULSE_SECONDS = 0.50;
    private static final double OUTTAKE_REVERSE_PULSE_POWER = -0.49;
    private static final double AUTO_LONG_TRIM_OFFSET_DEG = 3.0;
    private static final double AUTO_BLUE_LONG_PRELOAD_TRIM_OFFSET_DEG = 2.0;
    private static final double AUTO_RED_LONG_PRELOAD_TRIM_OFFSET_DEG = 5.0;
    private static final double AUTO_RED_LONG_TRIM_DELTA_DEG = 1.0;
    private static final double AUTO_BACKROW_LOOP_SHOOT_TRIM_OFFSET_DEG = 3.0;
    private static final double AUTO_CLOSE_FINAL_SHOOT_TRIM_DELTA_BLUE_DEG = 4.0;
    private static final double AUTO_CLOSE_FINAL_SHOOT_TRIM_DELTA_RED_DEG = -2.0;
    private static final double AUTO_CLOSE_HOOD_OFFSET = -0.1;
    private static final double AUTO_TOTAL_SECONDS = 30.0;
    private static final double FORCE_LEAVE_TIME_REMAINING_SECONDS = 1.0;
    private static final double AUTO_TELEMETRY_PERIOD_SEC = 0.20;

    private final Alliance alliance;
    private Range range;
    private boolean releaseAfterClosePickup;
    private boolean shootPreload;
    private boolean allowPickupCycles;
    @SuppressWarnings("unused")
    private boolean backRowLoopEnabled;
    private boolean closeLoopEnabled;
    private boolean forceLeaveAtOneSecond;
    private double backRowLoopPostIntakeHoldSeconds;
    private int backRowLoopCyclesTarget;
    private int backRowLoopCyclesCompleted;
    private AutoRoutePlanner routePlanner;
    private Range lastScoreRangeUsed;
    private enum PathRequest {
        GO_TO_PICKUP,
        COMPLETE_PICKUP,
        GO_TO_FAR_PICKUP_ZONE,
        BACKROW_COMPLETE_PICKUP_1,
        BACKROW_COMPLETE_PICKUP_2,
        GO_TO_SCORE,
        COMPLETE_RELEASE,
        LEAVE
    }

    // ===== State Machine =====
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.GO_TO_SHOOT;
    private StateMachine shootAllMachine;
    private StateMachine sortingShootAllMachine;
    private StateMachine slowShootAllMachine;
    private AutoTurretAim turretAim;
    private GoBildaPinpointDriver pinpoint;

    // ===== Robot and Subsystems =====
    private Robot robot;

    // ===== Runtime State =====
    private int[] rowSequence = new int[0];
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentAbsoluteRow = 1;
    private boolean preloadComplete = false;
    private AutoMotifTracker motifTracker;
    private int acquiredMotifId = -1;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime completeShootReadyTimer = new ElapsedTime();
    private final ElapsedTime motifAcquireTimer = new ElapsedTime();
    private final ElapsedTime goToPickupIdleTimer = new ElapsedTime();
    private final ElapsedTime closeLoopGoToPickupIdleTimer = new ElapsedTime();
    private final ElapsedTime closeLoopCompletePickupIdleTimer = new ElapsedTime();
    private final ElapsedTime farBackrowGoToPickupHoldTimer = new ElapsedTime();
    private final ElapsedTime backRowCompletePickupHoldTimer = new ElapsedTime();
    private final ElapsedTime intakeReversePulseTimer = new ElapsedTime();
    private AutoStates acquireMotifReturnState = AutoStates.GO_TO_SHOOT;
    private boolean motifResolvedThisAcquire = false;
    private boolean shootSequenceStarted = false;
    private boolean skipCurrentShot = false;
    private boolean shootStartedInGoToShoot = false;
    private boolean readyShootCommandedOnPath = false;
    private boolean intakeReversedOnShootPath = false;
    private boolean intakeResumedOnShootPath = false;
    private boolean outtakePulseAllowedOnShootPath = false;
    private boolean delayIntakeUntilPostPreload = false;
    private boolean goToPickupIdleSeen = false;
    private boolean closeLoopGoToPickupIdleSeen = false;
    private boolean closeLoopGoToPickupPart2Started = false;
    private boolean closeLoopCompletePickupIdleSeen = false;
    private boolean closeLoopCycleActive = false;
    private boolean farBackrowGoToPickupHoldSeen = false;
    private boolean backRowCompletePickupHoldSeen = false;
    private int backRowLoopEntryBallCount = 0;
    private boolean backRowLoopRetryUsed = false;
    private boolean forceOneMoreBackRowLoop = false;
    private boolean forceLeaveActivated = false;
    private boolean autoMachineStarted = false;

    protected BaseAuto(Alliance alliance) {
        this.alliance = alliance;
    }

    // ===== Shooter Timing Tuning =====
    public double clutchDownTime = 0.1;
    public double clutchDownFarTime = 0.3;
    public double spinTime = 2.75;
    public double spinUpTimeout = 1.75;

    // ===== FTC OpMode Lifecycle =====
    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();

        robot = new Robot(hardwareMap, telemetry);
        robot.useSorting = false;
        robot.intake.useSortingIntake = false;
        robot.intake.spindex.useSortingSpindex = false;
        AutoSpec spec = getSpec();
        range = spec.range;
        releaseAfterClosePickup = spec.releaseAfterClosePickup;
        shootPreload = spec.shootPreload;
        backRowLoopEnabled = spec.backRowLoopEnabled;
        closeLoopEnabled = spec.closeLoopEnabled;
        forceLeaveAtOneSecond = spec.forceLeaveAtOneSecond;
        backRowLoopPostIntakeHoldSeconds = spec.backRowLoopPostIntakeHoldSeconds;
        backRowLoopCyclesTarget = backRowLoopEnabled && !closeLoopEnabled
                ? Math.max(2, spec.backRowLoopCycles)
                : spec.backRowLoopCycles;
        rowSequence = spec.rowSequence;
        allowPickupCycles = rowSequence.length > 0;
        routePlanner = new AutoRoutePlanner(range);
        lastScoreRangeUsed = range;
        startPose = poses.findStartPose(alliance, range);
        delayIntakeUntilPostPreload = shootPreload;

        robot.other.drive.manualDrive = false;
        // Keep flywheel running throughout auto so shots can start faster.
        robot.outtake.shooter.useFlywheelPID = true;
        // Ensure shoot-while-moving lead compensation is active in auto.
        Outtake.enableMovingShotLead = true;
        // Auto should not inherit teleop baseline trim by default.
        Outtake.defaultTurretAimTrimOffsetDeg = 0.0;
        Outtake.turretAimTrimOffsetDeg = 0.0;
        robot.outtake.shooter.hoodOffset = range == Range.CLOSE_RANGE ? AUTO_CLOSE_HOOD_OFFSET : 0.0;
        if (delayIntakeUntilPostPreload) {
            // Keep intake idle until preload shooting is completed.
            robot.intake.autoIntake = false;
            robot.intake.spinner.autoSpin = false;
            robot.intake.spinner.setMegaSpinZero();
        }

        shootAllMachine = robot.getShootAllMachine();
        sortingShootAllMachine = robot.getSortedShootAllMachine();
        slowShootAllMachine = robot.getSlowShootAllMachine();
        turretAim = new AutoTurretAim(robot, poses, alliance, range, telemetry);
        motifTracker = null;

        autoMachine = buildAutoMachine();

        // Keep configured intakeSpeed value; do not override here.
        robot.outtake.vision.setRequiredTagId(alliance == Alliance.BLUE ? TAG_BLUE : TAG_RED);
        robot.outtake.vision.clearMotifTagId();

        if (alliance == Alliance.BLUE) {
            GlobalVariables.setAllianceColor(GlobalVariables.AllianceColor.BLUE);
        } else {
            GlobalVariables.setAllianceColor(GlobalVariables.AllianceColor.RED);
        }

        rowsToRun = rowSequence.length;
        rowsCompleted = 0;
        currentAbsoluteRow = (rowsToRun > 0) ? rowSequence[0] : routePlanner.getStartingAbsoluteRow();
        preloadComplete = false;
        backRowLoopCyclesCompleted = 0;
        backRowLoopEntryBallCount = 0;
        backRowLoopRetryUsed = false;
        forceOneMoreBackRowLoop = false;
        forceLeaveActivated = false;

        FollowerManager.initFollower(hardwareMap, startPose);
        GlobalVariables.setAutoFollowerValid(false);

        stateTimer.reset();
        telemetryTimer.reset();

    }

    @Override
    public void init_loop() {
        telemetryM.debug("Auto: " + this.getClass().getSimpleName() + " | State: " + activeState);
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        autoTimer.reset();
        autoMachineStarted = false;
        robot.toInit();
        robot.outtake.shooter.useFlywheelPID = true;
        robot.intake.autoIntake = false;
        robot.intake.spinner.autoSpin = false;
        robot.intake.spinner.setMegaSpinZero();
        robot.intake.clutch.spinClutchStop();
        Outtake.turretAimTrimOffsetDeg = getAutoTurretTrimOffsetForState();
        if (shootAllMachine != null) {
            shootAllMachine.start();
        }
        if (sortingShootAllMachine != null) {
            sortingShootAllMachine.start();
        }
        if (slowShootAllMachine != null) {
            slowShootAllMachine.start();
        }
    }

    @Override
    public void loop() {
        follower.update();

        if (!autoMachineStarted) {
            autoMachine.start();
            stateTimer.reset();
            autoMachineStarted = true;
        }

        if (autoMachineStarted) {
            if (shouldForceLeaveForMatchEnd()) {
                forceLeaveActivated = true;
                onEnterLeave();
            } else if (!forceLeaveActivated) {
                autoMachine.update();
            }
            maybeStartReadyShootAtPathProgress();
            updateIntakeDirectionAtShootPathProgress();
            maybeStartShootAtPathProgress();
        }
        Outtake.turretAimTrimOffsetDeg = getAutoTurretTrimOffsetForState();
        if (shouldSuppressTurretAimForCurrentState()) {
            suppressAutoTurretAim();
        } else {
            turretAim.updateAim(activeState, shouldAimObeliskDuringRow1Pickup(), getPreAimGoalPoseForCurrentState());
        }
        robot.update();
        stopIntakeInIfFull();
        PoseLoggingUtil.logMainPoseDetails(robot);
        if (shootAllMachine != null) {
            shootAllMachine.update();
        }
        if (sortingShootAllMachine != null) {
            sortingShootAllMachine.update();
        }
        if (slowShootAllMachine != null) {
            slowShootAllMachine.update();
        }
        if (telemetryTimer.seconds() >= AUTO_TELEMETRY_PERIOD_SEC) {
            telemetryM.debug("Auto: " + this.getClass().getSimpleName() + " | State: " + activeState);
            Pose scorePoseForNow = getScorePoseForCurrentShot();
            telemetryM.debug(String.format("Score Heading Cmd (deg): %.1f",
                    Math.toDegrees(scorePoseForNow.getHeading())));
            telemetryM.debug("Preload Complete: " + preloadComplete);
            telemetryM.debug("Alliance: " + alliance);
            if (robot != null && robot.outtake != null && robot.outtake.vision != null) {
                telemetryM.debug("Visible Tag ID: " + robot.outtake.vision.getCurrentTagId());
            }
//        telemetry.addData("Auto Action", getActionMessage());
//        telemetry.addData("State Time", "%.2f", stateTimer.seconds());
//        telemetry.addData("Current Row", currentAbsoluteRow);
//        telemetry.addData("Current Motif ID", acquiredMotifId);
//        telemetry.addData("Target RPM", robot.shooter.targetRPM);
//        telemetry.addData("Current RPM", robot.shooter.getShooterRPM());
//        telemetry.addData("Sees desired tag?", robot.shooter.hasDesiredTarget);
//        telemetry.addData("Turret Lock", robot.shooter.useTurretLock);
//        telemetry.addData("Ball List", robot.transfer.balls);
//        telemetry.addData("Shoot Order Number", robot.transfer.rotateOrder());
//        telemetry.addData("Vision Error", robot.vision.getTx());
//        telemetry.addData("ShootAll State", shootMachine.getState());
//        telemetry.addData("Turret Pow", robot.shooter.turret.getPower());
//        telemetry.addData("Is turret Tx In range?", Math.abs(robot.vision.getTx()) < 3);
//        telemetry.addData("Voltage", robot.getVoltage());
//        telemetry.addData("Intake Speed", intakeSpeed);


            telemetryM.update(telemetry);
            telemetry.update();
            telemetryTimer.reset();
        }

        drawCurrentAndHistory();
    }

    private boolean shouldForceLeaveForMatchEnd() {
        if (forceLeaveActivated) {
            return false;
        }
        if (!forceLeaveAtOneSecond) {
            return false;
        }
        if (activeState == AutoStates.LEAVE) {
            return false;
        }
        if (range == Range.CLOSE_RANGE) {
            return false;
        }
        double timeRemaining = AUTO_TOTAL_SECONDS - autoTimer.seconds();
        return timeRemaining <= FORCE_LEAVE_TIME_REMAINING_SECONDS;
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
        GlobalVariables.setAutoFollowerValid(follower != null);
    }

    // ===== State Machine Construction =====
    protected abstract StateMachine buildAutoMachine();
    protected abstract AutoSpec getSpec();

    protected final StateMachine buildStandardStateMachine() {
        return new StateMachineBuilder()
                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(this::shouldSkipShootPhase, AutoStates.LEAVE)
                .transition(this::pathReadyForNextAction, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldContinueActiveCloseLoop(), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldExitActiveCloseLoopToPickup(), AutoStates.GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldExitActiveCloseLoopToLeave(), AutoStates.LEAVE)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldGoToCloseLoopAfterShot(), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && !shouldStartNextCycle() && shouldEnterFarBackRowLoop(), AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldLeaveAfterShot(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
                .transition(this::goToPickupAdvanceReady, AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .onExit(this::onExitCompletePickup)
                .transition(() -> shouldReleaseAfterPickup() && pickupAdvanceReady(), AutoStates.COMPLETE_RELEASE)
                .transition(() -> !shouldReleaseAfterPickup() && pickupAdvanceReady(), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.COMPLETE_RELEASE)
                .onEnter(this::onEnterCompleteRelease)
                .onExit(this::onExitCompleteRelease)
                .transition(this::releasePathDone, AutoStates.RELEASE_WAIT)

                .state(AutoStates.RELEASE_WAIT)
                .onEnter(this::onEnterReleaseWait)
                .transition(this::releaseWaitDone, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterCloseLoopGoToPickup)
                .transition(this::backRowGoToPickupAdvanceReady, AutoStates.CLOSE_LOOP_COMPLETE_PICKUP)

                .state(AutoStates.CLOSE_LOOP_COMPLETE_PICKUP)
                .onEnter(this::onEnterCloseLoopCompletePickup)
                .transition(this::backRowCompletePickupAdvanceReady, AutoStates.CLOSE_LOOP_WAIT)

                .state(AutoStates.CLOSE_LOOP_WAIT)
                .onEnter(this::onEnterCloseLoopWait)
                .transition(() -> stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_IDLE_DELAY_SECONDS, AutoStates.CLOSE_LOOP_GO_TO_SHOOT)

                .state(AutoStates.CLOSE_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterCloseLoopGoToShoot)
                .transition(this::pathReadyForNextAction, AutoStates.CLOSE_LOOP_COMPLETE_SHOOT)

                .state(AutoStates.CLOSE_LOOP_COMPLETE_SHOOT)
                .onEnter(this::onEnterCloseLoopCompleteShoot)
                .onExit(this::onExitCloseLoopCompleteShoot)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldContinueActiveCloseLoop(), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldExitActiveCloseLoopToPickup(), AutoStates.GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldExitActiveCloseLoopToLeave(), AutoStates.LEAVE)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1)
                .onEnter(this::onEnterBackRowLoopCompletePickup1)
                .transition(this::backRowPickupHasThreeBallsAndPrepareShoot, AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .transition(this::backRowCompletePickup1PathDone, AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2)
                .onEnter(this::onEnterBackRowLoopCompletePickup2)
                .transition(this::backRowPickupHasThreeBallsAndPrepareShoot, AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .transition(this::backRowCompletePickup2Done, AutoStates.BACKROW_LOOP_GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterBackRowLoopGoToShoot)
                .onExit(this::onExitBackRowLoopCompleteShoot)
                .transition(this::shouldExitBackRowLoop, AutoStates.LEAVE)
                .transition(this::shouldContinueBackRowLoop, AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    protected final StateMachine buildPreloadOrMoveStateMachine() {
        return new StateMachineBuilder()
                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(this::shouldSkipShootPhase, AutoStates.LEAVE)
                .transition(this::pathReadyForNextAction, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> shootAdvanceReady() && pathReadyForNextAction() && shouldLeaveAfterShot(), AutoStates.LEAVE)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    // ===== State Machine Callbacks =====
    protected void onEnterAcquireMotif() {
        setActiveState(AutoStates.ACQUIRE_MOTIF);
        motifTracker.reset();
        motifAcquireTimer.reset();
        motifResolvedThisAcquire = false;
        acquireMotifReturnState = computePostAcquireTargetState();

        // Far auto: if motif is already visible, record immediately and advance.
        if (range == Range.LONG_RANGE && motifTracker.hasVisibleMotif()) {
            resolveMotifNow();
        }
    }

    protected boolean motifAcquiredOrTimedOut() {
        if (shouldBypassAcquireMotif()) {
            return true;
        }

        if (motifResolvedThisAcquire) {
            return true;
        }

        // During ACQUIRE_MOTIF, turretAim keeps commanding obelisk aim from current pose.
        if (motifTracker.hasVisibleMotif()) {
            resolveMotifNow();
            return true;
        }

        // If motif still not visible after aiming window, move on.
        if (motifAcquireTimer.seconds() >= MOTIF_ACQUIRE_AIM_WINDOW_SECONDS) {
            return true;
        }

        // Absolute timeout from ACQUIRE_MOTIF entry.
        return motifAcquireTimer.seconds() >= MOTIF_ACQUIRE_TIMEOUT;
    }

    protected void onExitAcquireMotif() {
        acquiredMotifId = motifTracker.getAcquiredMotifId();
    }

    protected void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);
        resetStateTimer();

        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }

        startPickupIntake();
        robot.outtake.shooter.useFlywheelPID = true;
        readyShootCommandedOnPath = false;
        intakeReversedOnShootPath = false;
        intakeResumedOnShootPath = false;
        outtakePulseAllowedOnShootPath = hasReachedPickupBallTarget();
        intakeReversePulseTimer.reset();
        shootStartedInGoToShoot = false;
        if (!preloadComplete && range == Range.LONG_RANGE) {
            robot.getReadyShoot();
            readyShootCommandedOnPath = true;
        }
        buildPath(PathRequest.GO_TO_SCORE);
        if (!preloadComplete && alliance == Alliance.RED && range == Range.CLOSE_RANGE) {
            followPath(goToScorePath, RED_CLOSE_PRELOAD_GO_TO_SHOOT_POWER);
            return;
        }
        followPath(goToScorePath);
    }

    protected void onEnterCompleteShoot() {
        stopIntakeForTravel();
        setActiveState(AutoStates.COMPLETE_SHOOT);
        resetStateTimer();
        shootTimer.reset();
        completeShootReadyTimer.reset();
        shootSequenceStarted = false;
        // Do not trust startup ball count for preload; only skip empty shots after preload.
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;
    }

    protected void onExitCompleteShoot() {
        if (closeLoopCycleActive) {
            backRowLoopCyclesCompleted++;
            return;
        }
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0; // start counting rows after preload
            shootStartedInGoToShoot = false;
            if (delayIntakeUntilPostPreload) {
                delayIntakeUntilPostPreload = false;
            }
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, rowsToRun);
        }
        // TODO: clear shoot command completion state.
    }

    protected void onEnterGoToPickup() {
        setActiveState(AutoStates.GO_TO_PICKUP);

        resetStateTimer();
        closeLoopCycleActive = false;
        goToPickupIdleSeen = false;
        goToPickupIdleTimer.reset();
        refreshCurrentAbsoluteRow();
        prepareSpindexForUpcomingPickup();
        startPickupIntake();

        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(goToPickupPath);
    }

    protected void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);
        resetStateTimer();
        startPickupIntake();

        buildPath(PathRequest.COMPLETE_PICKUP);
        if (currentAbsoluteRow == 4) {
            followPath(pickupPath, ROW4_COMPLETE_PICKUP_POWER);
        } else if (currentAbsoluteRow == 3) {
            followPath(pickupPath, ROW3_COMPLETE_PICKUP_POWER);
        } else {
            followPath(pickupPath, intakeSpeed);
        }
    }

    protected void onExitCompletePickup() {
        stopIntakeForTravel();
    }

    protected void onEnterCompleteRelease() {
        setActiveState(AutoStates.COMPLETE_RELEASE);

        resetStateTimer();
        stopIntakeForTravel();

        buildPath(PathRequest.COMPLETE_RELEASE);
        followPath(releaseCompletePath, RELEASE_COMPLETE_POWER);
    }

    protected void onExitCompleteRelease() {
    }

    protected void onEnterReleaseWait() {
        setActiveState(AutoStates.RELEASE_WAIT);
        resetStateTimer();
        stopIntakeForTravel();
    }

    protected void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);

        resetStateTimer();
        closeLoopCycleActive = false;

        // Park based on the range last used to score; fall back to the initially selected range.
        lastScoreRangeUsed = getLeaveRangeForLastShot();
        stopIntakeForTravel();
        disengageClutchForLeave();
        buildPath(PathRequest.LEAVE);
        followPath(leavePath);
    }

    protected void onEnterBackRowLoopGoToPickup() {
        setActiveState(AutoStates.CLOSE_LOOP_GO_TO_PICKUP);

        resetStateTimer();
        refreshCurrentAbsoluteRow();
        closeLoopGoToPickupIdleSeen = false;
        closeLoopGoToPickupIdleTimer.reset();
        closeLoopGoToPickupPart2Started = false;
        farBackrowGoToPickupHoldSeen = false;
        farBackrowGoToPickupHoldTimer.reset();
        prepareSpindexForUpcomingPickup();
        startPickupIntake();

        buildPath(PathRequest.GO_TO_FAR_PICKUP_ZONE);
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            followPath(backRowLoopPickupPath, CLOSE_LOOP_PICKUP_ZONE_POWER);
        } else {
            // Far back-row go-to-pickup: run the travel segment at full path power.
            followPath(backRowLoopPickupPath, FAR_PICKUP_ZONE_POWER);
        }
    }

    protected void onEnterCloseLoopGoToPickup() {
        onEnterBackRowLoopGoToPickup();
        closeLoopCycleActive = true;
        setActiveState(AutoStates.CLOSE_LOOP_GO_TO_PICKUP);
    }

    protected void onEnterBackRowLoopGoToShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_GO_TO_SHOOT);

        resetStateTimer();
        stopIntakeForTravel();
        backRowCompletePickupHoldSeen = false;
        backRowCompletePickupHoldTimer.reset();
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;
        robot.outtake.shooter.useFlywheelPID = true;
        readyShootCommandedOnPath = false;
        intakeReversedOnShootPath = false;
        intakeResumedOnShootPath = false;
        outtakePulseAllowedOnShootPath = !skipCurrentShot && hasReachedPickupBallTarget();
        intakeReversePulseTimer.reset();
        shootStartedInGoToShoot = false;
        shootTimer.reset();
        completeShootReadyTimer.reset();
        shootSequenceStarted = false;
        backRowLoopEntryBallCount = getLoadedBallCount();
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterCloseLoopGoToShoot() {
        onEnterBackRowLoopGoToShoot();
        setActiveState(AutoStates.CLOSE_LOOP_GO_TO_SHOOT);
    }

    protected void onEnterBackRowLoopCompletePickup1() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1);

        resetStateTimer();
        prepareSpindexForUpcomingPickup();
        startPickupIntake();

        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP_1);
        followPath(backRowLoopCompletePickupPath, BACKROW_COMPLETE_PICKUP_POWER);
    }

    protected void onEnterBackRowLoopCompletePickup2() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2);

        resetStateTimer();
        startPickupIntake();

        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP_2);
        followPath(backRowLoopCompletePickupPath, BACKROW_COMPLETE_PICKUP_POWER);
    }

    protected void onEnterCloseLoopCompletePickup() {
        setActiveState(AutoStates.CLOSE_LOOP_COMPLETE_PICKUP);

        resetStateTimer();
        closeLoopCompletePickupIdleSeen = false;
        closeLoopCompletePickupIdleTimer.reset();
        startPickupIntake();

        Pose currentPose = (follower != null) ? follower.getPose() : null;
        PathChain part2Path = pathLibrary.closeLoopPickupPart2(currentPose, alliance);
        followPath(part2Path, CLOSE_LOOP_PICKUP_PART2_POWER);
    }

    protected void onEnterCloseLoopWait() {
        setActiveState(AutoStates.CLOSE_LOOP_WAIT);
        resetStateTimer();
        startPickupIntake();
    }

    protected void onEnterBackRowLoopCompleteShoot() {
        stopIntakeForTravel();
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT);
        resetStateTimer();
        shootTimer.reset();
        completeShootReadyTimer.reset();
        shootSequenceStarted = false;
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;
        backRowLoopEntryBallCount = getLoadedBallCount();

        // Run the same shoot sequence logic used by COMPLETE_SHOOT for back-row loop shots.
    }

    protected void onEnterCloseLoopCompleteShoot() {
        onEnterBackRowLoopCompleteShoot();
        setActiveState(AutoStates.CLOSE_LOOP_COMPLETE_SHOOT);
    }

    protected void onExitBackRowLoopCompleteShoot() {
        int ballsAfter = getLoadedBallCount();
        int ballsShotThisCycle = Math.max(0, backRowLoopEntryBallCount - ballsAfter);
        if (backRowLoopCyclesCompleted == 0 && ballsShotThisCycle == 0 && !backRowLoopRetryUsed) {
            // If first back-row loop shot didn't fire anything, force one extra retry before leaving.
            forceOneMoreBackRowLoop = true;
            backRowLoopRetryUsed = true;
        }
        backRowLoopCyclesCompleted++;
    }

    protected void onExitCloseLoopCompleteShoot() {
        onExitBackRowLoopCompleteShoot();
    }

    // ===== Path Building =====
    protected void buildPath(PathRequest request) {
        if (follower == null) {
            return;
        }

        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return;
        }

        switch (request) {
            case GO_TO_PICKUP:
                goToPickupPath = buildGoToPickupPath(currentPose);
                break;
            case COMPLETE_PICKUP:
                pickupPath = buildPickupPath(currentPose);
                break;
            case GO_TO_FAR_PICKUP_ZONE:
                backRowLoopPickupPath = buildBackRowLoopGoToPickupPath(currentPose);
                break;
            case BACKROW_COMPLETE_PICKUP_1:
                backRowLoopCompletePickupPath = buildBackRowLoopCompletePickup1Path(currentPose);
                break;
            case BACKROW_COMPLETE_PICKUP_2:
                backRowLoopCompletePickupPath = buildBackRowLoopCompletePickup2Path(currentPose);
                break;
            case GO_TO_SCORE:
                lastScoreRangeUsed = getScoreRangeForCurrentShot();
                if (activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                        || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT) {
                    goToScorePath = buildBackRowLoopGoToScorePath(currentPose);
                } else {
                    goToScorePath = buildGoToScorePath(currentPose);
                }
                break;
            case COMPLETE_RELEASE:
                releaseCompletePath = buildReleaseCompletePath(currentPose);
                break;
            case LEAVE:
                leavePath = buildLeavePath(currentPose);
                break;
        }
    }

    protected PathChain buildGoToPickupPath(Pose currentPose) {
        if (currentAbsoluteRow == 4) {
            return pathLibrary.buildLinearTwoStep(
                    currentPose,
                    poses.getRow4GoToPickup(alliance),
                    poses.getRow4IntermediatePickup(alliance)
            );
        }
        if (shouldUseCurvedRow2GoToPickup()
                && range == Range.CLOSE_RANGE
                && currentAbsoluteRow == 2) {
            return pathLibrary.row2GoToPickup(currentPose, alliance, range);
        }
        return pathLibrary.goToPickup(currentPose, alliance, range, currentAbsoluteRow);
    }

    protected boolean shouldUseCurvedRow2GoToPickup() {
        return false;
    }

    protected PathChain buildPickupPath(Pose currentPose) {
        if (currentAbsoluteRow == 4) {
            return pathLibrary.row4CompletePickup(currentPose, alliance);
        }
        return pathLibrary.pickup(currentPose, alliance, range, currentAbsoluteRow);
    }

    protected PathChain buildGoToScorePath(Pose currentPose) {
        Pose scorePose = getScorePoseForCurrentShot();
        if (!preloadComplete && range == Range.LONG_RANGE) {
            return null;
        }
        if (range == Range.LONG_RANGE) {
            return pathLibrary.goToScore(currentPose, scorePose);
        }
        if (closeLoopCycleActive && range == Range.CLOSE_RANGE) {
            return pathLibrary.closeLoopGoToShoot(currentPose, alliance, getBackRowLoopScorePoseForCurrentShot(), false);
        }
        boolean isRow2GoToShoot = currentAbsoluteRow == 2;
        boolean closeNonFinalShot = range == Range.CLOSE_RANGE
                && preloadComplete
                && shouldStartNextCycle();
        if (closeNonFinalShot) {
            if (isRow2GoToShoot) {
                return pathLibrary.row2GoToShoot(currentPose, alliance, scorePose);
            }
            return pathLibrary.closeLoopGoToShoot(currentPose, alliance, scorePose, false);
        }
        return pathLibrary.goToScore(currentPose, scorePose);
    }

    protected PathChain buildFarPickupZonePath(Pose currentPose) {
        double yOffset = getBackRowLoopYOffsetIn();
        if (yOffset != 0.0) {
            return pathLibrary.buildCurve(
                    currentPose,
                    offsetPoseY(poses.getFarPickupZoneControl(alliance), yOffset),
                    offsetPoseY(poses.getFarPickupZone(alliance), yOffset)
            );
        }
        return pathLibrary.farPickupZone(currentPose, alliance);
    }

    protected PathChain buildBackRowLoopGoToPickupPath(Pose currentPose) {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return pathLibrary.closeLoopPickup(currentPose, alliance);
        }
        return buildFarPickupZonePath(currentPose);
    }

    protected PathChain buildBackRowLoopCompletePickup1Path(Pose currentPose) {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return pathLibrary.closeLoopPickupPart2(currentPose, alliance);
        }
        return pathLibrary.backRowLoopCompletePickup1(currentPose, alliance);
    }

    protected PathChain buildBackRowLoopCompletePickup2Path(Pose currentPose) {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return pathLibrary.closeLoopPickupPart2(currentPose, alliance);
        }
        return pathLibrary.backRowLoopCompletePickup2(currentPose, alliance);
    }

    protected PathChain buildBackRowLoopGoToScorePath(Pose currentPose) {
        Pose scorePose = getBackRowLoopScorePoseForCurrentShot();
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            boolean isFinalLoopShot = shouldExitBackRowLoop();
            return pathLibrary.closeLoopGoToShoot(currentPose, alliance, scorePose, isFinalLoopShot);
        }
        if (range == Range.LONG_RANGE) {
            return pathLibrary.goToScore(currentPose, scorePose);
        }
        return pathLibrary.goToScore(currentPose, scorePose);
    }

    protected boolean isGoToShootPathState() {
        return activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT;
    }

    protected PathChain buildReleaseCompletePath(Pose currentPose) {
        return pathLibrary.releaseComplete(currentPose, alliance, range);
    }

    protected PathChain buildLeavePath(Pose currentPose) {
        return pathLibrary.leave(currentPose, alliance, lastScoreRangeUsed);
    }

    // ===== Row and Shot Planning =====
    protected void refreshCurrentAbsoluteRow() {
        if (rowSequence.length == 0) {
            currentAbsoluteRow = routePlanner.getStartingAbsoluteRow();
            return;
        }
        int idx = preloadComplete ? rowsCompleted : 0;
        int clampedIdx = Math.max(0, Math.min(idx, rowSequence.length - 1));
        currentAbsoluteRow = rowSequence[clampedIdx];
    }

    protected Range getScoreRangeForCurrentShot() {
        return routePlanner.getScoreRangeForShot(preloadComplete, currentAbsoluteRow);
    }

    protected Range getLeaveRangeForLastShot() {
        return routePlanner.getLeaveRangeForLastShot(preloadComplete, rowsCompleted);
    }

    protected Pose getScorePoseForCurrentShot() {
        Range scoreRange = getScoreRangeForCurrentShot();
        Pose base = poses.getScore(alliance, scoreRange);
        if (!preloadComplete && range == Range.LONG_RANGE && startPose != null) {
            return startPose;
        }
        if (scoreRange == Range.CLOSE_RANGE && preloadComplete && currentAbsoluteRow == 2) {
            // Keep row-2 close shots on the dedicated row-2 pose; do not override with final-shot pose.
            Pose row2Pose = poses.getRow2ShootClose(alliance);
            return new Pose(row2Pose.getX(), row2Pose.getY(), Math.toRadians(0.0));
        }
        if (scoreRange == Range.CLOSE_RANGE
                && preloadComplete
                && rowsCompleted > 0
                && currentAbsoluteRow == 1
                && !shouldStartNextCycle()
                && !shouldGoToCloseLoopAfterShot()) {
            // Final close shot should happen from the deeper field pose (same location as close leave).
            return poses.getFinalShootClose(alliance);
        }
        if (scoreRange == Range.CLOSE_RANGE && preloadComplete) {
            double headingDeg = 0.0;
            if (alliance == Alliance.RED && currentAbsoluteRow == 1) {
                // Use the tuned row-1 red heading from the pose constants, not the 180-deg default.
                headingDeg = Math.toDegrees(poses.getFinalShootClose(alliance).getHeading());
            }
            return new Pose(base.getX(), base.getY(), Math.toRadians(headingDeg));
        }
        return base;
    }

    // ===== Follower Helpers =====
    protected void followPath(PathChain path) {
        if (follower != null && path != null) {
            follower.followPath(path, true);
        }
    }

    protected void followPath(PathChain path, double power) {
        if (follower != null && path != null) {
            follower.followPath(path, power, true);
        }
    }

    protected void prepareForShootWhileDriving() {
        if (robot == null || robot.outtake == null || robot.outtake.shooter == null) {
            return;
        }
        robot.outtake.shooter.useFlywheelPID = true;
        robot.getReadyShoot();
    }

    protected void maybeStartReadyShootAtPathProgress() {
        if (readyShootCommandedOnPath) {
            return;
        }
        boolean shootPathState = activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT;
        if (!shootPathState) {
            return;
        }
        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }
        if (shouldSkipBackRowLoopShot()) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null || robot == null) {
            return;
        }
        double pathT = follower.getCurrentPath().getClosestPointTValue();
        double requiredProgress = getLoadedBallCount() >= PICKUP_TARGET_BALL_COUNT
                ? READY_SHOOT_FULL_PATH_PROGRESS
                : READY_SHOOT_PARTIAL_PATH_PROGRESS;
        if (!Double.isFinite(pathT) || pathT < requiredProgress) {
            return;
        }
        robot.getReadyShoot();
        readyShootCommandedOnPath = true;
    }

    protected void updateIntakeDirectionAtShootPathProgress() {
        boolean shootPathState = activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT;
        if (!shootPathState) {
            return;
        }
        if (!preloadComplete) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }
        if (!outtakePulseAllowedOnShootPath) {
            return;
        }
        double pathT = follower.getCurrentPath().getClosestPointTValue();
        if (!intakeReversedOnShootPath) {
            if (!Double.isFinite(pathT) || pathT < OUTTAKE_REVERSE_PULSE_START_PATH_PROGRESS) {
                return;
            }
            reverseIntakeForShootTravel();
            intakeReversedOnShootPath = true;
            intakeReversePulseTimer.reset();
            return;
        }

        if (intakeResumedOnShootPath) {
            return;
        }

        if (intakeReversePulseTimer.seconds() >= OUTTAKE_REVERSE_PULSE_SECONDS) {
            if (!hasReachedPickupBallTarget()) {
                startPickupIntake();
            } else {
                stopIntakeForTravel();
            }
            intakeResumedOnShootPath = true;
        }
    }

    protected void reverseIntakeForShootTravel() {
        if (robot == null || robot.intake == null || robot.intake.spinner == null || robot.intake.clutch == null) {
            return;
        }
        robot.intake.autoIntake = false;
        robot.intake.spinner.autoSpin = false;
        robot.intake.spinner.setMegaSpinPow(OUTTAKE_REVERSE_PULSE_POWER);
        robot.intake.clutch.spinClutchStop();
    }

    protected void startPickupIntake() {
        if (robot == null || robot.intake == null || robot.intake.spinner == null || robot.intake.clutch == null) {
            return;
        }
        if (hasReachedPickupBallTarget()) {
            stopMainIntakeOnly();
            return;
        }
        robot.intake.autoIntake = true;
        robot.intake.spinner.autoSpin = true;
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();
    }

    protected void stopIntakeInIfFull() {
        if (!hasReachedPickupBallTarget()) {
            return;
        }
        if (activeState == AutoStates.COMPLETE_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_COMPLETE_SHOOT) {
            return;
        }
        if (intakeReversedOnShootPath && !intakeResumedOnShootPath) {
            return;
        }
        stopMainIntakeOnly();
    }

    protected void prepareSpindexForUpcomingPickup() {
        if (robot == null || robot.intake == null || robot.intake.spindex == null) {
            return;
        }
        boolean useForwardOne = alliance == Alliance.BLUE
                ? currentAbsoluteRow == 2
                : currentAbsoluteRow != 2;
        if (useForwardOne) {
            robot.intake.spindex.setSpindexForwardOne();
            return;
        }
        robot.intake.spindex.setSpindexBackwardOne();
    }

    protected void maintainSpindexForGoToPickup() {
        if (!isGoToPickupSetupState()) {
            return;
        }
        prepareSpindexForUpcomingPickup();
    }

    protected boolean isStandardGoToPickupState() {
        return activeState == AutoStates.GO_TO_PICKUP;
    }

    protected boolean isGoToPickupSetupState() {
        return activeState == AutoStates.GO_TO_PICKUP
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_PICKUP;
    }

    protected boolean shouldSuppressTurretAimForCurrentState() {
        return closeLoopEnabled
                && range == Range.CLOSE_RANGE
                && (activeState == AutoStates.CLOSE_LOOP_GO_TO_PICKUP
                || activeState == AutoStates.CLOSE_LOOP_COMPLETE_PICKUP
                || activeState == AutoStates.CLOSE_LOOP_WAIT);
    }

    protected void suppressAutoTurretAim() {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null) {
            return;
        }
        robot.outtake.setPreventTurretWrap(true);
        robot.outtake.setAimLockEnabled(false);
        robot.outtake.turret.turretVelocity = 0;
    }

    protected void stopIntakeForTravel() {
        if (robot == null || robot.intake == null || robot.intake.spinner == null) {
            return;
        }
        stopMainIntakeOnly();
        if (robot.intake.clutch != null) {
            robot.intake.clutch.spinClutchStop();
        }
    }

    protected void disengageClutchForLeave() {
        if (robot == null || robot.intake == null || robot.intake.clutch == null) {
            return;
        }
        robot.intake.clutch.setClutchUp();
        robot.intake.clutch.spinClutchStop();
    }

    protected void stopMainIntakeOnly() {
        if (robot == null || robot.intake == null || robot.intake.spinner == null) {
            return;
        }
        robot.intake.autoIntake = false;
        robot.intake.spinner.autoSpin = false;
        robot.intake.spinner.setMegaSpinZero();
    }

    protected boolean followerIdle() {
        return follower != null && !follower.isBusy();
    }

    protected boolean pathReadyForNextAction() {
        return pathReadyForProgress(getPathAdvanceProgressForCurrentState());
    }

    protected double getPathAdvanceProgressForCurrentState() {
        if (isGoToShootPathState()) {
            return GO_TO_SHOOT_PATH_ADVANCE_PROGRESS;
        }
        if (activeState == AutoStates.GO_TO_PICKUP) {
            return GO_TO_PICKUP_PATH_ADVANCE_PROGRESS;
        }
        return PATH_ADVANCE_PROGRESS;
    }

    protected boolean pathReadyForProgress(double requiredProgress) {
        if (followerIdle()) {
            return true;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return false;
        }
        if (follower.getFollowingPathChain()
                && follower.getCurrentPathChain() != null
                && follower.getChainIndex() < follower.getCurrentPathChain().size() - 1) {
            return false;
        }
        double pathT = follower.getCurrentPath().getClosestPointTValue();
        return Double.isFinite(pathT) && pathT >= requiredProgress;
    }

    // ===== Decision Helpers =====
    protected boolean shouldShootPreload() {
        return shootPreload;
    }

    /** Returns true if we should detour to the lever after collecting the first row in sequence. */
    protected boolean shouldReleaseAfterPickup() {
        boolean justFinishedFirstRow = preloadComplete && rowsCompleted == 0;
        return allowPickupCycles && justFinishedFirstRow && releaseAfterClosePickup;
    }

    /** Determines if another row remains after the current shot completes. */
    protected boolean shouldStartNextCycle() {
        if (!preloadComplete) {
            return rowsToRun > 0;
        }
        int currentIndex = getCurrentRowSequenceIndex();
        return currentIndex >= 0 && currentIndex < rowsToRun - 1;
    }

    protected int getCurrentRowSequenceIndex() {
        for (int i = 0; i < rowsToRun; i++) {
            if (rowSequence[i] == currentAbsoluteRow) {
                return i;
            }
        }
        return -1;
    }

    protected boolean shouldSkipShootPhase() {
        return !preloadComplete && !shouldShootPreload();
    }

    protected boolean shouldAcquireMotifAfterPreloadShot() {
        // Close auto resolves motif during first pickup after preload.
        // Long auto only re-acquires after preload if motif is still unresolved.
        boolean motifAlreadyResolved = acquiredMotifId == AutoMotifTracker.TAG_MOTIF_1
                || acquiredMotifId == AutoMotifTracker.TAG_MOTIF_2
                || acquiredMotifId == AutoMotifTracker.TAG_MOTIF_3;
        return range == Range.LONG_RANGE && !preloadComplete && !motifAlreadyResolved;
    }

    protected boolean shouldBypassAcquireMotif() {
        // Close auto should only acquire once, right after preload completes.
        return range == Range.CLOSE_RANGE && !preloadComplete;
    }

    protected boolean shouldEnterBackRowLoop() {
        return backRowLoopEnabled;
    }

    protected boolean shouldEnterCloseLoop() {
        return shouldEnterBackRowLoop() && closeLoopEnabled && range == Range.CLOSE_RANGE;
    }

    protected boolean shouldEnterFarBackRowLoop() {
        return shouldEnterBackRowLoop() && !shouldEnterCloseLoop();
    }

    protected boolean shouldGoToCloseLoopAfterShot() {
        return preloadComplete
                && !closeLoopCycleActive
                && shouldEnterCloseLoop()
                && rowsCompleted == 0
                && backRowLoopCyclesCompleted < backRowLoopCyclesTarget;
    }

    protected boolean shouldContinueActiveCloseLoop() {
        return closeLoopCycleActive && shouldContinueBackRowLoop();
    }

    protected boolean shouldExitActiveCloseLoopToPickup() {
        return closeLoopCycleActive && shouldExitBackRowLoop() && hasPendingPickupRow();
    }

    protected boolean shouldExitActiveCloseLoopToLeave() {
        return closeLoopCycleActive && shouldExitBackRowLoop() && !hasPendingPickupRow() && shouldLeaveAfterShot();
    }

    protected boolean shouldLeaveAfterShot() {
        if (range == Range.CLOSE_RANGE && !shouldStartNextCycle() && !shouldEnterFarBackRowLoop()) {
            return false;
        }
        return !shouldStartNextCycle() && !shouldEnterFarBackRowLoop();
    }

    protected boolean hasPendingPickupRow() {
        return preloadComplete && rowsCompleted < rowsToRun;
    }

    protected boolean pickupAdvanceReady() {
        if (hasReachedPickupBallTarget()) {
            return true;
        }
        if (activeState == AutoStates.COMPLETE_PICKUP && currentAbsoluteRow == 1) {
            if (stateTimer.seconds() >= ROW1_PICKUP_TIMEOUT_SECONDS) {
                return true;
            }
        }
        if (activeState == AutoStates.COMPLETE_PICKUP && currentAbsoluteRow == 4) {
            return row4PickupTimedOut() || pathReadyForNextAction();
        }
        return pathReadyForNextAction() || row4PickupTimedOut();
    }

    protected boolean goToPickupAdvanceReady() {
        if (hasReachedPickupBallTarget()) {
            return true;
        }
        if (stateTimedOut()) {
            return true;
        }
        if (pathReadyForNextAction()) {
            if (!goToPickupIdleSeen) {
                goToPickupIdleSeen = true;
                goToPickupIdleTimer.reset();
            }
            return goToPickupIdleTimer.seconds() >= GO_TO_PICKUP_IDLE_HOLD_SECONDS;
        }
        goToPickupIdleSeen = false;
        return false;
    }

    protected boolean backRowGoToPickupAdvanceReady() {
        if (hasReachedPickupBallTarget()) {
            return true;
        }
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return followerIdle()
                    || stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS;
        }
        boolean pickupTimedOut = stateTimer.seconds() >= BACKROW_PICKUP_TIMEOUT_SECONDS;
        if (pickupTimedOut) {
            return true;
        }
        if (!pathReadyForNextAction()) {
            farBackrowGoToPickupHoldSeen = false;
            return false;
        }
        if (!farBackrowGoToPickupHoldSeen) {
            farBackrowGoToPickupHoldSeen = true;
            farBackrowGoToPickupHoldTimer.reset();
        }
        return farBackrowGoToPickupHoldTimer.seconds() >= FAR_BACKROW_GO_TO_PICKUP_IDLE_HOLD_SECONDS;
    }

    protected boolean backRowCompletePickupAdvanceReady() {
        boolean ready;
        if (hasReachedPickupBallTarget()) {
            ready = true;
        } else if (stateTimedOut()) {
            ready = true;
        } else if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            ready = followerIdle()
                    || stateTimer.seconds() >= CLOSE_LOOP_COMPLETE_PICKUP_TIMEOUT_SECONDS;
        } else {
            ready = pathReadyForNextAction() || backRowPickupTimedOut();
        }
        return backRowCompletePickupHoldSatisfied(ready);
    }

    protected boolean backRowPickupHasThreeBallsAndPrepareShoot() {
        if (!hasReachedPickupBallTarget()) {
            return false;
        }
        if (follower != null) {
            follower.breakFollowing();
        }
        stopIntakeForTravel();
        return true;
    }

    protected boolean backRowCompletePickup1PathDone() {
        return pathReadyForNextAction() || backRowPickupTimedOut();
    }

    protected boolean backRowCompletePickup2Done() {
        return pathReadyForNextAction() || backRowPickupTimedOut();
    }

    protected boolean backRowCompletePickupHoldSatisfied(boolean ready) {
        if (!ready) {
            backRowCompletePickupHoldSeen = false;
            return false;
        }
        if (backRowLoopPostIntakeHoldSeconds <= 0.0) {
            return true;
        }
        if ((activeState == AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2)
                && currentAbsoluteRow == 4) {
            return true;
        }
        if (!backRowCompletePickupHoldSeen) {
            backRowCompletePickupHoldSeen = true;
            backRowCompletePickupHoldTimer.reset();
        }
        return backRowCompletePickupHoldTimer.seconds() >= backRowLoopPostIntakeHoldSeconds;
    }

    protected boolean row4PickupTimedOut() {
        if (!(activeState == AutoStates.COMPLETE_PICKUP && currentAbsoluteRow == 4)) {
            return false;
        }
        return stateTimer.seconds() >= ROW4_PICKUP_TIMEOUT_SECONDS;
    }

    protected boolean backRowPickupTimedOut() {
        if (activeState != AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1
                && activeState != AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2
                && activeState != AutoStates.CLOSE_LOOP_COMPLETE_PICKUP) {
            return false;
        }
        return stateTimer.seconds() >= BACKROW_PICKUP_TIMEOUT_SECONDS;
    }

    protected boolean hasReachedPickupBallTarget() {
        return getLoadedBallCount() >= PICKUP_TARGET_BALL_COUNT;
    }

    protected boolean isHeadingSettledForGoToPickup() {
        Pose target = poses.getPickupStart(alliance, range, currentAbsoluteRow);
        return isHeadingWithinTolerance(target, PICKUP_HEADING_TOLERANCE_DEG);
    }

    protected boolean isHeadingWithinTolerance(Pose targetPose, double toleranceDeg) {
        if (follower == null || targetPose == null) {
            return false;
        }
        Pose current = follower.getPose();
        if (current == null) {
            return false;
        }
        double headingErrorDeg = Math.abs(Math.toDegrees(Math.atan2(
                Math.sin(targetPose.getHeading() - current.getHeading()),
                Math.cos(targetPose.getHeading() - current.getHeading())
        )));
        return headingErrorDeg <= toleranceDeg;
    }

    protected int getLoadedBallCount() {
        if (robot == null) return 0;
        return robot.getLoadedBallCount();
    }

    protected AutoStates computePostAcquireTargetState() {
        if (preloadComplete) {
            if (shouldStartNextCycle()) {
                return AutoStates.GO_TO_PICKUP;
            }
            if (shouldEnterCloseLoop()) {
                return AutoStates.CLOSE_LOOP_GO_TO_PICKUP;
            }
            if (shouldEnterFarBackRowLoop()) {
                return AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1;
            }
            return AutoStates.LEAVE;
        }
        return AutoStates.GO_TO_SHOOT;
    }

    protected boolean shouldReturnToGoToShootAfterAcquire() {
        return acquireMotifReturnState == AutoStates.GO_TO_SHOOT;
    }

    protected boolean shouldReturnToGoToPickupAfterAcquire() {
        return acquireMotifReturnState == AutoStates.GO_TO_PICKUP;
    }

    protected boolean shouldReturnToCloseLoopPickupAfterAcquire() {
        return acquireMotifReturnState == AutoStates.CLOSE_LOOP_GO_TO_PICKUP;
    }

    protected boolean shouldReturnToBackrowPickupAfterAcquire() {
        return acquireMotifReturnState == AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1;
    }

    protected boolean shouldReturnToLeaveAfterAcquire() {
        return acquireMotifReturnState == AutoStates.LEAVE;
    }

    protected void resolveMotifNow() {
        motifTracker.resolveMotif(preloadComplete);
        acquiredMotifId = motifTracker.getAcquiredMotifId();
        motifResolvedThisAcquire = true;
    }

    protected void maybeResolveMotifDuringFirstPickupAfterPreload() {
        if (motifTracker == null) {
            return;
        }
        if (acquiredMotifId == AutoMotifTracker.TAG_MOTIF_1
                || acquiredMotifId == AutoMotifTracker.TAG_MOTIF_2
                || acquiredMotifId == AutoMotifTracker.TAG_MOTIF_3) {
            return;
        }
        if (!isFirstCompletePickupAfterPreload()) {
            return;
        }
        if (motifTracker.hasVisibleMotif()) {
            resolveMotifNow();
        }
    }

    protected boolean isRow2PickupAfterPreload() {
        boolean pickupState = activeState == AutoStates.GO_TO_PICKUP
                || activeState == AutoStates.COMPLETE_PICKUP;
        return pickupState && preloadComplete && currentAbsoluteRow == 2;
    }

    protected boolean isFirstCompletePickupAfterPreload() {
        return activeState == AutoStates.COMPLETE_PICKUP
                && preloadComplete
                && rowsCompleted == 0;
    }

    protected boolean shouldAimObeliskDuringRow1Pickup() {
        return isFirstCompletePickupAfterPreload();
    }

    protected Pose getPreAimGoalPoseForCurrentState() {
        if (alliance != Alliance.RED
                || range != Range.CLOSE_RANGE
                || !isRow2PickupAfterPreload()) {
            return null;
        }
        return getScorePoseForCurrentShot();
    }

    // ===== State/Timer Helpers =====
    protected boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    protected boolean shootTimedOut() {
        return shootTimer.seconds() >= SHOOT_ACTION_SECONDS;
    }

    protected boolean shootAdvanceReady() {
        // Keep timeout fallback so a jammed mechanism cannot deadlock the auto.
        if (shootSequenceStarted && shootTimedOut()) {
            return true;
        }
        if (!shootSequenceStarted && stateTimedOut()) {
            return true;
        }
        return shootActionComplete();
    }

    protected boolean allShotBallsCleared() {
        return skipCurrentShot || getLoadedBallCount() <= 0;
    }

    protected boolean shouldSkipBackRowLoopShot() {
        return preloadComplete
                && (activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_SHOOT)
                && (skipCurrentShot || getLoadedBallCount() <= 0);
    }

    protected boolean shootActionComplete() {
        if (!preloadComplete && !shouldShootPreload()) {
            return true;
        }
        if (skipCurrentShot) {
            return true;
        }
        if (!shootSequenceStarted) {
            if (shouldStartShootSequence()) {
                robot.forceShootAllThreeOnNextStart = !preloadComplete;
                armAutoShootSequence();
                shootSequenceStarted = true;
                shootTimer.reset();
            }
            return false;
        }
        StateMachine activeShootMachine = getAutoShootMachine();
        if (activeShootMachine == null) {
            return true;
        }
        return isAutoShootMachineIdle();

        //TODO get a boolean from shooter subsystem
    }

    private void armAutoShootSequence() {
        robot.initShootAllMachine = false;
        robot.initSlowShootAllMachine = false;
        robot.initSortedShootAllMachine = false;

        StateMachine activeShootMachine = getAutoShootMachine();
        if (activeShootMachine == sortingShootAllMachine) {
            robot.initSortedShootAllMachine = true;
        } else if (activeShootMachine == slowShootAllMachine) {
            robot.initSlowShootAllMachine = true;
        } else {
            robot.initShootAllMachine = true;
        }
    }

    protected void maybeStartShootAtPathProgress() {
        if (!SHOOT_WHILE_MOVING_ENABLED) {
            return;
        }
        if (shootStartedInGoToShoot) {
            return;
        }
        boolean shootPathState = activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT;
        if (!shootPathState) {
            return;
        }
        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }
        if (shouldSkipBackRowLoopShot()) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }
        double pathT = follower.getCurrentPath().getClosestPointTValue();
        double requiredProgress = SHOOT_START_PATH_PROGRESS;
        if (!Double.isFinite(pathT) || pathT < requiredProgress) {
            return;
        }
        // Let COMPLETE_SHOOT arm the shooter through the normal, reliable shoot flow.
        shootStartedInGoToShoot = true;
    }

    private StateMachine getAutoShootMachine() {
        if (robot != null && robot.useSorting) {
            return sortingShootAllMachine;
        }
        if (shouldUseFastShootByDistance()) {
            return shootAllMachine;
        }
        return slowShootAllMachine;
    }

    private boolean shouldUseFastShootByDistance() {
        return robot != null
                && robot.outtake != null
                && robot.outtake.distanceInches < Outtake.longRangeFastShotMinDistanceInches;
    }

    private boolean isAutoShootMachineIdle() {
        StateMachine activeShootMachine = getAutoShootMachine();
        if (activeShootMachine == sortingShootAllMachine) {
            return sortingShootAllMachine != null
                    && sortingShootAllMachine.getState() == Robot.SortedShootAllStates.INIT
                    && !robot.initSortedShootAllMachine;
        }
        if (activeShootMachine == slowShootAllMachine) {
            return slowShootAllMachine != null
                    && slowShootAllMachine.getState() == Robot.SlowShootAllStates.INIT
                    && !robot.initSlowShootAllMachine;
        }
        return shootAllMachine != null
                && shootAllMachine.getState() == Robot.ShootAllStates.INIT
                && !robot.initShootAllMachine;
    }

    protected boolean shouldStartShootSequence() {
        return true;
    }

    protected void setActiveState(AutoStates state) {
        activeState = state;
    }

    private double getAutoTurretTrimOffsetForState() {
        boolean shootState = activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.COMPLETE_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_COMPLETE_SHOOT;
        if (shootState
                && range == Range.CLOSE_RANGE
                && preloadComplete) {
            double closeShootTrimOffsetDeg = getCloseShootTrimOffsetForCurrentRow();
            if (isFinalCloseShoot()) {
                return closeShootTrimOffsetDeg + getFinalCloseShootTrimDeltaDeg();
            }
            return closeShootTrimOffsetDeg;
        }
        if (activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_COMPLETE_SHOOT) {
            return AUTO_BACKROW_LOOP_SHOOT_TRIM_OFFSET_DEG + getRedLongTrimDeltaDeg();
        }
        if (range == Range.LONG_RANGE) {
            if (!preloadComplete) {
                double preloadTrim = alliance == Alliance.RED
                        ? AUTO_RED_LONG_PRELOAD_TRIM_OFFSET_DEG
                        : AUTO_BLUE_LONG_PRELOAD_TRIM_OFFSET_DEG;
                return preloadTrim + getRedLongTrimDeltaDeg();
            }
            return AUTO_LONG_TRIM_OFFSET_DEG + getRedLongTrimDeltaDeg();
        }
        return 0.0;
    }

    private double getRedLongTrimDeltaDeg() {
        return range == Range.LONG_RANGE && alliance == Alliance.RED
                ? AUTO_RED_LONG_TRIM_DELTA_DEG
                : 0.0;
    }

    private boolean isFinalCloseShoot() {
        return range == Range.CLOSE_RANGE
                && preloadComplete
                && isFinalPlannedRowShot()
                && !shouldStartNextCycle()
                && !shouldGoToCloseLoopAfterShot();
    }

    private double getCloseShootTrimOffsetForCurrentRow() {
        return 0.0;
    }

    private double getFinalCloseShootTrimDeltaDeg() {
        return alliance == Alliance.RED
                ? AUTO_CLOSE_FINAL_SHOOT_TRIM_DELTA_RED_DEG
                : AUTO_CLOSE_FINAL_SHOOT_TRIM_DELTA_BLUE_DEG;
    }

    private boolean isFinalPlannedRowShot() {
        if (rowsToRun <= 0 || rowSequence.length == 0) {
            return false;
        }
        int finalIndex = Math.min(rowsToRun, rowSequence.length) - 1;
        return currentAbsoluteRow == rowSequence[finalIndex];
    }

    protected void resetStateTimer() {
        stateTimer.reset();
    }

    /** Waits 1s after the release path finishes before advancing. */
    protected boolean releaseWaitDone() {
        return stateTimer.seconds() >= RELEASE_IDLE_SECONDS;
    }

    protected boolean releasePathDone() {
        return pathReadyForNextAction() || stateTimer.seconds() >= RELEASE_TIMEOUT_SECONDS;
    }

    protected boolean backRowLoopShootComplete() {
        return pathReadyForNextAction() && shootAdvanceReady();
    }

    protected boolean shouldExitBackRowLoop() {
        if (!backRowLoopShootComplete()) {
            return false;
        }
        if (range == Range.LONG_RANGE) {
            return false;
        }
        if (shouldForceRetryAfterFirstLongBackRowNoShot()) {
            forceOneMoreBackRowLoop = true;
            backRowLoopRetryUsed = true;
            return false;
        }
        if (forceOneMoreBackRowLoop) {
            return false;
        }
        if (backRowLoopCyclesTarget <= 0) {
            return false;
        }
        return (backRowLoopCyclesCompleted + 1) >= backRowLoopCyclesTarget;
    }

    protected boolean shouldContinueBackRowLoop() {
        if (!backRowLoopShootComplete()) {
            return false;
        }
        if (range == Range.LONG_RANGE) {
            return true;
        }
        if (shouldForceRetryAfterFirstLongBackRowNoShot()) {
            forceOneMoreBackRowLoop = false;
            backRowLoopRetryUsed = true;
            return true;
        }
        if (forceOneMoreBackRowLoop) {
            forceOneMoreBackRowLoop = false;
            return true;
        }
        if (backRowLoopCyclesTarget <= 0) {
            return true;
        }
        return (backRowLoopCyclesCompleted + 1) < backRowLoopCyclesTarget;
    }

    protected Pose getBackRowLoopScorePoseForCurrentShot() {
        Pose scorePose = getScorePoseForCurrentShot();
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            Pose preloadPose = poses.getScore(alliance, Range.CLOSE_RANGE);
            return new Pose(preloadPose.getX(), preloadPose.getY(), scorePose.getHeading());
        }
        return scorePose;
    }

    protected double getBackRowLoopYOffsetIn() {
        return (backRowLoopCyclesCompleted % 2 == 0) ? 0.0 : 12.0;
    }

    protected Pose offsetPoseY(Pose pose, double yOffset) {
        if (pose == null || yOffset == 0.0) {
            return pose;
        }
        return new Pose(pose.getX(), pose.getY() + yOffset, pose.getHeading());
    }

    private boolean shouldForceRetryAfterFirstLongBackRowNoShot() {
        if (range != Range.LONG_RANGE) {
            return false;
        }
        if (backRowLoopRetryUsed || backRowLoopCyclesCompleted != 0) {
            return false;
        }
        int ballsAfter = getLoadedBallCount();
        int ballsShotThisCycle = Math.max(0, backRowLoopEntryBallCount - ballsAfter);
        return ballsShotThisCycle == 0;
    }

    // ===== Telemetry Helpers =====
    protected String getCurrentShotLabel() {
        if (!preloadComplete) {
            return "preload";
        }
        return "row " + currentAbsoluteRow;
    }

    protected String getActionMessage() {
        switch (activeState) {
            case ACQUIRE_MOTIF:
                return "Looking for motif tag";
            case GO_TO_SHOOT:
                return "Go to shoot spot (" + getCurrentShotLabel() + ")";
            case COMPLETE_SHOOT:
                return "Shooting " + getCurrentShotLabel();
            case GO_TO_PICKUP:
                return "Approach intake spot (row " + currentAbsoluteRow + ")";
            case COMPLETE_PICKUP:
                return "Complete intake (row " + currentAbsoluteRow + ")";
            case GO_TO_RELEASE:
                return "Approach release spot";
            case COMPLETE_RELEASE:
                return "Complete release";
            case BACKROW_LOOP_COMPLETE_PICKUP_1:
                return "Back-row loop: complete pickup 1";
            case BACKROW_LOOP_COMPLETE_PICKUP_2:
                return "Back-row loop: complete pickup 2";
            case BACKROW_LOOP_GO_TO_SHOOT:
                return "Back-row loop: go to shoot spot / shoot";
            case BACKROW_LOOP_COMPLETE_SHOOT:
                return "Back-row loop: shooting";
            case CLOSE_LOOP_GO_TO_PICKUP:
                return "Close loop: go to pickup";
            case CLOSE_LOOP_COMPLETE_PICKUP:
                return "Close loop: complete pickup";
            case CLOSE_LOOP_WAIT:
                return "Close loop: hold";
            case CLOSE_LOOP_GO_TO_SHOOT:
                return "Close loop: go to shoot";
            case CLOSE_LOOP_COMPLETE_SHOOT:
                return "Close loop: shooting";
            case LEAVE:
                return "Parking / leave path";
            default:
                return "Idle";
        }
    }
}
