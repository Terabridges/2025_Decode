package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;


import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrent;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.telemetryM;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoIntakeSpeed;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoMotifTracker;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPathLibrary;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoRoutePlanner;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoTurretAim;
import org.firstinspires.ftc.teamcode.config.autoUtil.ReleaseWaiter;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

import org.firstinspires.ftc.teamcode.config.utility.PoseLoggingUtil;

@PsiKitAutoLog(rlogPort = 5802)
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
    private double intakeSpeed = 0.25;

//    private final AutoIntakeSpeed intakeSpeedModel = new AutoIntakeSpeed(
//            -0.02, 0.47, 0.18, 0.24, -0.01, 0.01);

    // ===== Constants =====
    private static final double SHOOT_ACTION_SECONDS = 5.0;
    private static final double COMPLETE_SHOOT_MIN_SETTLE_SECONDS = 0.25;
    private static final double COMPLETE_SHOOT_READY_TIMEOUT_SECONDS = 2.0;
    private static final double COMPLETE_SHOOT_TURRET_TOLERANCE_DEG = 2.0;
    private static final double SHOOT_SETTLE_MAX_TRANSLATIONAL_SPEED_IN_S = 1.5;
    private static final double SHOOT_SETTLE_MAX_ANGULAR_SPEED_DEG_S = 12.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 1.5;
    private static final double MOTIF_ACQUIRE_AIM_WINDOW_SECONDS = 0.5;
    private static final double STATE_TIMEOUT_SECONDS = 4.0; // fallback: force state advance after this time
    private static final double GO_TO_PICKUP_IDLE_HOLD_SECONDS = 0.1;
    private static final double GO_TO_PICKUP_SLOWDOWN_START_T = 0.60;
    private static final double GO_TO_PICKUP_SLOWDOWN_POWER = 0.60;
    private static final int GO_TO_PICKUP_SLOWDOWN_MAX_ROW = 2;
    private static final double GO_TO_SHOOT_SLOWDOWN_START_T = 0.60;
    private static final double GO_TO_SHOOT_SLOWDOWN_POWER = 0.60;
    private static final double BACKROW_COMPLETE_PICKUP_SLOWDOWN_START_T = 0.60;
    private static final double BACKROW_COMPLETE_PICKUP_SLOWDOWN_POWER = 0.60;
    private static final double BACKROW_GO_TO_PICKUP_SLOWDOWN_START_T = 0.60;
    private static final double BACKROW_GO_TO_PICKUP_SLOWDOWN_POWER = 0.75;
    private static final double ROW4_PICKUP_TIMEOUT_SECONDS = 3.5;
    private static final double BACKROW_PICKUP_TIMEOUT_SECONDS = 1.5;
    private static final double FAR_PICKUP_ZONE_POWER = 0.75;
    private static final double FAR_BACKROW_REPOSITION_POWER = 0.5;
    private static final double FAR_BACKROW_REPOSITION_BACK_DELTA_X = 8.0;
    private static final double FAR_BACKROW_REPOSITION_FORWARD_DELTA_X = 4.0;
    private static final double CLOSE_LOOP_PICKUP_ZONE_POWER = 1.0;
    private static final double CLOSE_LOOP_PICKUP_PART2_POWER = 0.75;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_POWER = 0.75;
    private static final double BACKROW_COMPLETE_PICKUP_POWER = 1.0;
    private static final double ROW4_COMPLETE_PICKUP_POWER = 0.60;
    private static final double ROW4_INTERMEDIATE_PICKUP_POWER = 0.75;
    private static final double ROW4_GO_TO_PICKUP_HOLD_SECONDS = 0.1;
    private static final double ROW4_COMPLETE_PICKUP_HOLD_SECONDS = 0.2;
    private static final double ROW4_PRE_SHOOT_HEADING_DEG_BLUE = 25.0;
    private static final double CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS = 1.05;
    private static final double CLOSE_LOOP_GO_TO_PICKUP_IDLE_DELAY_SECONDS = 1.05;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_TIMEOUT_SECONDS = 1.5;
    private static final double PICKUP_HEADING_TOLERANCE_DEG = 3.0;
    private static final int PICKUP_TARGET_BALL_COUNT = 3;
    private static final int TAG_BLUE = 20;
    private static final int TAG_RED = 24;
    private static final double RELEASE_IDLE_SECONDS = 1.0;
    private static final double RELEASE_TIMEOUT_SECONDS = 1.5;
    private static final double RELEASE_COMPLETE_POWER = 0.75;
    private static final double PRELOAD_SHOOT_START_PATH_PROGRESS = 0.85;
    private static final double SHOOT_START_PATH_PROGRESS = 0.85;
    private static final double LONG_RANGE_GO_TO_SHOOT_END_PROGRESS_T = 0.90;

    private final Alliance alliance;
    private Range range;
    private boolean releaseAfterClosePickup;
    private boolean shootPreload;
    private boolean allowPickupCycles;
    @SuppressWarnings("unused")
    private boolean backRowLoopEnabled;
    private boolean closeLoopEnabled;
    private int backRowLoopCyclesTarget;
    private int backRowLoopCyclesCompleted;
    private AutoRoutePlanner routePlanner;
    private Range lastScoreRangeUsed;
    private enum PathRequest {
        GO_TO_PICKUP,
        COMPLETE_PICKUP,
        GO_TO_FAR_PICKUP_ZONE,
        BACKROW_COMPLETE_PICKUP,
        GO_TO_SCORE,
        COMPLETE_RELEASE,
        LEAVE
    }

    // ===== State Machine =====
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;
    private StateMachine shootAllMachine;
    private StateMachine sortingShootAllMachine;
    private AutoTurretAim turretAim;

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
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime completeShootReadyTimer = new ElapsedTime();
    private final ElapsedTime motifAcquireTimer = new ElapsedTime();
    private final ElapsedTime goToPickupIdleTimer = new ElapsedTime();
    private final ElapsedTime row4CompletePickupHoldTimer = new ElapsedTime();
    private final ElapsedTime closeLoopGoToPickupIdleTimer = new ElapsedTime();
    private ReleaseWaiter releaseWaiter = new ReleaseWaiter(RELEASE_IDLE_SECONDS);
    private AutoStates acquireMotifReturnState = AutoStates.GO_TO_SHOOT;
    private boolean motifResolvedThisAcquire = false;
    private boolean shootSequenceStarted = false;
    private boolean skipCurrentShot = false;
    private boolean shootStartedInGoToShoot = false;
    private boolean delayIntakeUntilPostPreload = false;
    private boolean goToPickupIdleSeen = false;
    private boolean goToPickupSlowdownApplied = false;
    private boolean goToShootSlowdownApplied = false;
    private boolean backRowCompletePickupSlowdownApplied = false;
    private boolean backRowGoToPickupSlowdownApplied = false;
    private boolean farBackrowRetreatStarted = false;
    private boolean farBackrowForwardStarted = false;
    private boolean row4CompletePickupIdleSeen = false;
    private boolean row4IntermediatePickupStarted = false;
    private boolean row4PreShootHeadingAlignStarted = false;
    private boolean backRowGoToShootReverseActive = false;
    private boolean closeLoopGoToPickupIdleSeen = false;
    private boolean closeLoopGoToPickupPart2Started = false;

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
        robot = new Robot(hardwareMap, telemetry);
        AutoSpec spec = getSpec();
        range = spec.range;
        releaseAfterClosePickup = spec.releaseAfterClosePickup;
        shootPreload = spec.shootPreload;
        backRowLoopEnabled = spec.backRowLoopEnabled;
        closeLoopEnabled = spec.closeLoopEnabled;
        backRowLoopCyclesTarget = spec.backRowLoopCycles;
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
        // Keep recoil-comp code available, but disable it during auto runtime.
        Outtake.enableRpmRecoilComp = false;
        if (delayIntakeUntilPostPreload) {
            // Keep intake idle until preload shooting is completed.
            robot.intake.autoIntake = false;
            robot.intake.spinner.autoSpin = false;
            robot.intake.spinner.setMegaSpinZero();
        }

        shootAllMachine = robot.getShootAllMachine();
        sortingShootAllMachine = robot.getSortedShootAllMachine();
        turretAim = new AutoTurretAim(robot, poses, alliance, range, telemetry);
        motifTracker = new AutoMotifTracker(robot, alliance, range, MOTIF_ACQUIRE_TIMEOUT);

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

        FollowerManager.initFollower(hardwareMap, startPose);
        GlobalVariables.setAutoFollowerValid(false);

        stateTimer.reset();

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
        autoMachine.start();
        robot.toInit();
        if (shootAllMachine != null) {
            shootAllMachine.start();
        }
        if (sortingShootAllMachine != null) {
            sortingShootAllMachine.start();
        }
    }

    @Override
    public void loop() {
        follower.update();

        autoMachine.update();
        maybeStartGoToPickupSlowdown();
        maybeStartBackRowGoToPickupSlowdown();
        maybeStartBackRowCompletePickupSlowdown();
        maybeStartGoToShootSlowdown();
        maybeReverseIntakeLateLongRangeGoToShoot();
        maybeStartShootAtPathProgress();
        turretAim.updateAim(activeState, shouldAimObeliskDuringRow1Pickup());
        robot.update();
        maybeResolveMotifDuringFirstPickupAfterPreload();
        PoseLoggingUtil.logMainPoseDetails(robot);
        if (shootAllMachine != null) {
            shootAllMachine.update();
        }
        if (sortingShootAllMachine != null) {
            sortingShootAllMachine.update();
        }

        telemetryM.debug("Auto: " + this.getClass().getSimpleName() + " | State: " + activeState);
        telemetryM.debug("Motif Pattern: " + GlobalVariables.getMotif());
        telemetryM.debug("Motif Tag ID: " + acquiredMotifId);
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

        drawCurrentAndHistory();
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
                .state(AutoStates.ACQUIRE_MOTIF)
                .onEnter(this::onEnterAcquireMotif)
                .onExit(this::onExitAcquireMotif)
                .transition(this::shouldBypassAcquireMotif, AutoStates.GO_TO_SHOOT)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToGoToShootAfterAcquire(), AutoStates.GO_TO_SHOOT)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToGoToPickupAfterAcquire(), AutoStates.GO_TO_PICKUP)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToCloseLoopPickupAfterAcquire(), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToBackrowPickupAfterAcquire(), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToLeaveAfterAcquire(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(this::shouldSkipShootPhase, AutoStates.LEAVE)
                .transition(this::followerIdle, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> shootAdvanceReady() && followerIdle() && shouldAcquireMotifAfterPreloadShot(), AutoStates.ACQUIRE_MOTIF)
                .transition(() -> shootAdvanceReady() && followerIdle() && shouldGoToCloseLoopAfterShot(), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && followerIdle() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && followerIdle() && !shouldStartNextCycle() && shouldEnterFarBackRowLoop(), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .transition(() -> shootAdvanceReady() && followerIdle() && !shouldStartNextCycle() && !shouldEnterFarBackRowLoop(), AutoStates.LEAVE)

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
                .transition(this::releaseWaitDone, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterCloseLoopGoToPickup)
                .transition(this::backRowGoToPickupAdvanceReady, AutoStates.CLOSE_LOOP_COMPLETE_PICKUP)

                .state(AutoStates.CLOSE_LOOP_COMPLETE_PICKUP)
                .onEnter(this::onEnterCloseLoopCompletePickup)
                .transition(this::backRowCompletePickupAdvanceReady, AutoStates.CLOSE_LOOP_GO_TO_SHOOT)

                .state(AutoStates.CLOSE_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterCloseLoopGoToShoot)
                .transition(this::followerIdle, AutoStates.CLOSE_LOOP_COMPLETE_SHOOT)

                .state(AutoStates.CLOSE_LOOP_COMPLETE_SHOOT)
                .onEnter(this::onEnterCloseLoopCompleteShoot)
                .onExit(this::onExitCloseLoopCompleteShoot)
                .transition(() -> shouldExitBackRowLoop() && hasPendingPickupRow(), AutoStates.GO_TO_PICKUP)
                .transition(() -> shouldExitBackRowLoop() && !hasPendingPickupRow(), AutoStates.LEAVE)
                .transition(this::shouldContinueBackRowLoop, AutoStates.CLOSE_LOOP_GO_TO_PICKUP)

                .state(AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterBackRowLoopGoToPickup)
                .transition(this::backRowGoToPickupAdvanceReady, AutoStates.BACKROW_LOOP_COMPLETE_PICKUP)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP)
                .onEnter(this::onEnterBackRowLoopCompletePickup)
                .transition(this::backRowCompletePickupAdvanceReady, AutoStates.BACKROW_LOOP_GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterBackRowLoopGoToShoot)
                .transition(this::followerIdle, AutoStates.BACKROW_LOOP_COMPLETE_SHOOT)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT)
                .onEnter(this::onEnterBackRowLoopCompleteShoot)
                .onExit(this::onExitBackRowLoopCompleteShoot)
                .transition(this::shouldExitBackRowLoop, AutoStates.LEAVE)
                .transition(this::shouldContinueBackRowLoop, AutoStates.BACKROW_LOOP_GO_TO_PICKUP)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    protected final StateMachine buildPreloadOrMoveStateMachine() {
        return new StateMachineBuilder()
                .state(AutoStates.ACQUIRE_MOTIF)
                .onEnter(this::onEnterAcquireMotif)
                .onExit(this::onExitAcquireMotif)
                .transition(this::shouldBypassAcquireMotif, AutoStates.GO_TO_SHOOT)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToGoToShootAfterAcquire(), AutoStates.GO_TO_SHOOT)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToLeaveAfterAcquire(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(this::shouldSkipShootPhase, AutoStates.LEAVE)
                .transition(this::followerIdle, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> shootAdvanceReady() && followerIdle() && shouldAcquireMotifAfterPreloadShot(), AutoStates.ACQUIRE_MOTIF)
                .transition(() -> shootAdvanceReady() && followerIdle(), AutoStates.LEAVE)

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
        goToShootSlowdownApplied = false;

        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }

        ensureIntakeRunningForGoToShoot();
        prepareForShootWhileDriving();
        shootStartedInGoToShoot = false;
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);
        resetStateTimer();
        shootTimer.reset();
        completeShootReadyTimer.reset();
        shootSequenceStarted = shootStartedInGoToShoot;
        // Do not trust startup ball count for preload; only skip empty shots after preload.
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;
    }

    protected void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0; // start counting rows after preload
            shootStartedInGoToShoot = false;
            if (delayIntakeUntilPostPreload) {
                robot.intake.spinner.autoSpin = true;
                robot.intake.autoIntake = true;
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
        goToPickupIdleSeen = false;
        goToPickupSlowdownApplied = false;
        goToPickupIdleTimer.reset();
        row4IntermediatePickupStarted = false;
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        refreshCurrentAbsoluteRow();
        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(goToPickupPath);
    }

    protected void maybeStartGoToPickupSlowdown() {
        if (activeState != AutoStates.GO_TO_PICKUP || goToPickupSlowdownApplied) {
            return;
        }
        boolean applyLongRangeSmoothing = range == Range.LONG_RANGE;
        if (!applyLongRangeSmoothing && currentAbsoluteRow > GO_TO_PICKUP_SLOWDOWN_MAX_ROW) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }

        double pathT = follower.getCurrentPath().getClosestPointTValue();
        if (!Double.isFinite(pathT) || pathT < GO_TO_PICKUP_SLOWDOWN_START_T) {
            return;
        }

        Pose currentPose = follower.getPose();
        PathChain finalApproachPath = buildGoToPickupPath(currentPose);
        if (finalApproachPath == null) {
            return;
        }
        followPath(finalApproachPath, GO_TO_PICKUP_SLOWDOWN_POWER);
        goToPickupSlowdownApplied = true;
        goToPickupIdleSeen = false;
        goToPickupIdleTimer.reset();
    }

    protected void maybeStartGoToShootSlowdown() {
        if (goToShootSlowdownApplied || !isLongRangeGoToShootState()) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }

        double pathT = follower.getCurrentPath().getClosestPointTValue();
        if (!Double.isFinite(pathT) || pathT < GO_TO_SHOOT_SLOWDOWN_START_T) {
            return;
        }

        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return;
        }

        PathChain finalApproachPath = buildLongRangeGoToShootPath(currentPose);
        followPath(finalApproachPath, GO_TO_SHOOT_SLOWDOWN_POWER);
        goToShootSlowdownApplied = true;
    }

    protected void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);
        resetStateTimer();
        row4CompletePickupIdleSeen = false;
        row4PreShootHeadingAlignStarted = false;
        row4CompletePickupHoldTimer.reset();
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        buildPath(PathRequest.COMPLETE_PICKUP);
        // TODO: run intake command while completing pickup path.
        if (currentAbsoluteRow == 4) {
            followPath(pickupPath, ROW4_COMPLETE_PICKUP_POWER);
        } else {
            followPath(pickupPath, intakeSpeed);
        }
    }

    protected void onExitCompletePickup() {
        // TODO: stop intake command after pickup completes.
    }

    protected void onEnterCompleteRelease() {
        setActiveState(AutoStates.COMPLETE_RELEASE);

        resetStateTimer();
        releaseWaiter.reset();

        buildPath(PathRequest.COMPLETE_RELEASE);
        followPath(releaseCompletePath, RELEASE_COMPLETE_POWER);
    }

    protected void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);

        resetStateTimer();

        // Park based on the range last used to score; fall back to the initially selected range.
        lastScoreRangeUsed = getLeaveRangeForLastShot();
        buildPath(PathRequest.LEAVE);
        followPath(leavePath);
    }

    protected void onEnterBackRowLoopGoToPickup() {
        setActiveState(AutoStates.BACKROW_LOOP_GO_TO_PICKUP);

        resetStateTimer();
        closeLoopGoToPickupIdleSeen = false;
        closeLoopGoToPickupIdleTimer.reset();
        closeLoopGoToPickupPart2Started = false;
        backRowGoToPickupSlowdownApplied = false;
        farBackrowRetreatStarted = false;
        farBackrowForwardStarted = false;
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        buildPath(PathRequest.GO_TO_FAR_PICKUP_ZONE);
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            followPath(backRowLoopPickupPath, CLOSE_LOOP_PICKUP_ZONE_POWER);
        } else {
            // Far back-row go-to-pickup: run full segment at requested 75% speed.
            followPath(backRowLoopPickupPath, FAR_PICKUP_ZONE_POWER);
        }
    }

    protected void onEnterCloseLoopGoToPickup() {
        onEnterBackRowLoopGoToPickup();
        setActiveState(AutoStates.CLOSE_LOOP_GO_TO_PICKUP);
    }

    protected void onEnterBackRowLoopGoToShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_GO_TO_SHOOT);

        resetStateTimer();
        goToShootSlowdownApplied = false;

        ensureIntakeRunningForGoToShoot();
        // Stop intake transfer and pre-stage shot while driving to score.
        prepareForShootWhileDriving();
        shootStartedInGoToShoot = false;
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterCloseLoopGoToShoot() {
        onEnterBackRowLoopGoToShoot();
        setActiveState(AutoStates.CLOSE_LOOP_GO_TO_SHOOT);
    }

    protected void onEnterBackRowLoopCompletePickup() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP);

        resetStateTimer();
        backRowCompletePickupSlowdownApplied = false;
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP);
        followPath(backRowLoopCompletePickupPath, BACKROW_COMPLETE_PICKUP_POWER);
    }

    protected void onEnterCloseLoopCompletePickup() {
        setActiveState(AutoStates.CLOSE_LOOP_COMPLETE_PICKUP);

        resetStateTimer();
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP);
        followPath(backRowLoopCompletePickupPath, CLOSE_LOOP_COMPLETE_PICKUP_POWER);
    }

    protected void onEnterBackRowLoopCompleteShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT);

        resetStateTimer();
        shootTimer.reset();
        completeShootReadyTimer.reset();
        // Preserve whether shooting already started during GO_TO_SHOOT path progress.
        shootSequenceStarted = shootStartedInGoToShoot;
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;

        // Run the same shoot sequence logic used by COMPLETE_SHOOT for back-row loop shots.
    }

    protected void onEnterCloseLoopCompleteShoot() {
        onEnterBackRowLoopCompleteShoot();
        setActiveState(AutoStates.CLOSE_LOOP_COMPLETE_SHOOT);
    }

    protected void onExitBackRowLoopCompleteShoot() {
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
            case BACKROW_COMPLETE_PICKUP:
                backRowLoopCompletePickupPath = buildBackRowLoopCompletePickupPath(currentPose);
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
            return pathLibrary.buildLinear(currentPose, poses.getRow4GoToPickup(alliance));
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

    protected void maybeStartBackRowCompletePickupSlowdown() {
        if (activeState != AutoStates.BACKROW_LOOP_COMPLETE_PICKUP
                || backRowCompletePickupSlowdownApplied) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }

        double pathT = follower.getCurrentPath().getClosestPointTValue();
        if (!Double.isFinite(pathT) || pathT < BACKROW_COMPLETE_PICKUP_SLOWDOWN_START_T) {
            return;
        }

        Pose currentPose = follower.getPose();
        PathChain finalApproachPath = buildBackRowLoopCompletePickupPath(currentPose);
        if (finalApproachPath == null) {
            return;
        }
        followPath(finalApproachPath, BACKROW_COMPLETE_PICKUP_SLOWDOWN_POWER);
        backRowCompletePickupSlowdownApplied = true;
    }

    protected void maybeStartBackRowGoToPickupSlowdown() {
        if (activeState != AutoStates.BACKROW_LOOP_GO_TO_PICKUP
                || (closeLoopEnabled && range == Range.CLOSE_RANGE)
                || backRowGoToPickupSlowdownApplied) {
            return;
        }
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }

        double pathT = follower.getCurrentPath().getClosestPointTValue();
        if (!Double.isFinite(pathT) || pathT < BACKROW_GO_TO_PICKUP_SLOWDOWN_START_T) {
            return;
        }

        Pose currentPose = follower.getPose();
        PathChain finalApproachPath = buildBackRowLoopGoToPickupPath(currentPose);
        followPath(finalApproachPath, BACKROW_GO_TO_PICKUP_SLOWDOWN_POWER);
        backRowGoToPickupSlowdownApplied = true;
    }

    protected void maybeReverseIntakeLateLongRangeGoToShoot() {
        if (robot == null || robot.intake == null || robot.intake.spinner == null) {
            return;
        }

        boolean longRangeShootPathState = range == Range.LONG_RANGE
                && (activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT);

        if (!longRangeShootPathState) {
            if (backRowGoToShootReverseActive) {
                robot.intake.autoIntake = true;
                robot.intake.spinner.autoSpin = true;
                robot.intake.spinner.setMegaSpinIn();
                backRowGoToShootReverseActive = false;
            }
            return;
        }

        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }

        double pathT = follower.getCurrentPath().getClosestPointTValue();
        if (!Double.isFinite(pathT)) {
            return;
        }

        if (pathT >= 0.60) {
            if (!backRowGoToShootReverseActive) {
                // Reverse intake for the final 40% of back-row go-to-shoot.
                robot.intake.autoIntake = false;
                robot.intake.spinner.autoSpin = true;
                robot.intake.spinner.overrideSpinZero();
                robot.intake.spinner.setMegaSpinOut();
                backRowGoToShootReverseActive = true;
            }
        } else if (backRowGoToShootReverseActive) {
            robot.intake.autoIntake = true;
            robot.intake.spinner.autoSpin = true;
            robot.intake.spinner.setMegaSpinIn();
            backRowGoToShootReverseActive = false;
        }
    }

    protected PathChain buildGoToScorePath(Pose currentPose) {
        Pose scorePose = getScorePoseForCurrentShot();
        if (range == Range.LONG_RANGE) {
            return pathLibrary.goToScoreAtProgress(currentPose, scorePose, LONG_RANGE_GO_TO_SHOOT_END_PROGRESS_T);
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
        return pathLibrary.farPickupZone(currentPose, alliance);
    }

    protected PathChain buildBackRowLoopGoToPickupPath(Pose currentPose) {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return pathLibrary.closeLoopPickup(currentPose, alliance);
        }
        return buildFarPickupZonePath(currentPose);
    }

    protected PathChain buildBackRowLoopCompletePickupPath(Pose currentPose) {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return pathLibrary.closeLoopCompletePickup(currentPose, alliance);
        }
        // Far back-row loop does movement in GO_TO_PICKUP + hold; no extra complete-pickup path.
        return null;
    }

    protected PathChain buildBackRowLoopGoToScorePath(Pose currentPose) {
        Pose scorePose = getBackRowLoopScorePoseForCurrentShot();
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            boolean isFinalLoopShot = shouldExitBackRowLoop();
            return pathLibrary.closeLoopGoToShoot(currentPose, alliance, scorePose, isFinalLoopShot);
        }
        if (range == Range.LONG_RANGE) {
            return pathLibrary.goToScoreAtProgress(currentPose, scorePose, LONG_RANGE_GO_TO_SHOOT_END_PROGRESS_T);
        }
        return pathLibrary.goToScore(currentPose, scorePose);
    }

    protected PathChain buildLongRangeGoToShootPath(Pose currentPose) {
        if (currentPose == null) {
            return null;
        }
        if (activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT) {
            return buildBackRowLoopGoToScorePath(currentPose);
        }
        return buildGoToScorePath(currentPose);
    }

    protected boolean isLongRangeGoToShootState() {
        if (range != Range.LONG_RANGE) {
            return false;
        }
        return activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT;
    }

    protected PathChain buildReleaseCompletePath(Pose currentPose) {
        if (range == Range.CLOSE_RANGE) {
            return pathLibrary.buildLinear(currentPose, poses.getReleaseComplete(alliance, range));
        }
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
        int finalRowInSequence = rowSequence[Math.max(0, rowsToRun - 1)];
        boolean isFinalPlannedRowShot = currentAbsoluteRow == finalRowInSequence;
        if (scoreRange == Range.CLOSE_RANGE
                && preloadComplete
                && isFinalPlannedRowShot
                && !shouldStartNextCycle()
                && !shouldGoToCloseLoopAfterShot()) {
            return poses.getFinalShootClose(alliance);
        }
        if (scoreRange == Range.CLOSE_RANGE && preloadComplete) {
            // Close-range cycle shots run at 180 deg for both alliances.
            double headingDeg = 180.0;
            if (currentAbsoluteRow == 2) {
                Pose row2Pose = poses.getRow2ShootClose(alliance);
                return new Pose(row2Pose.getX(), row2Pose.getY(), Math.toRadians(headingDeg));
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
        //robot.getReadyShoot();
    }

    protected void ensureIntakeRunningForGoToShoot() {
        if (robot == null || robot.intake == null || robot.intake.spinner == null || robot.intake.clutch == null) {
            return;
        }
        robot.intake.autoIntake = true;
        robot.intake.spinner.autoSpin = true;
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();
    }

    protected boolean followerIdle() {
        return follower != null && !follower.isBusy();
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
        // rowsCompleted will be incremented when exiting COMPLETE_SHOOT; look ahead one.
        int nextCount = rowsCompleted + 1;
        return nextCount < rowsToRun;
    }

    protected boolean shouldSkipShootPhase() {
        return !preloadComplete && !shouldShootPreload();
    }

    protected boolean shouldAcquireMotifAfterPreloadShot() {
        return range == Range.CLOSE_RANGE && !preloadComplete;
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
                && shouldEnterCloseLoop()
                && rowsCompleted == 0
                && backRowLoopCyclesCompleted < backRowLoopCyclesTarget;
    }

    protected boolean hasPendingPickupRow() {
        return preloadComplete && rowsCompleted < rowsToRun;
    }

    protected boolean pickupAdvanceReady() {
        if (activeState == AutoStates.COMPLETE_PICKUP && currentAbsoluteRow == 4) {
            if (row4PickupTimedOut()) {
                return true;
            }
            if (followerIdle()) {
                if (!row4CompletePickupIdleSeen) {
                    row4CompletePickupIdleSeen = true;
                    row4CompletePickupHoldTimer.reset();
                }
                if (!row4PreShootHeadingAlignStarted) {
                    Pose currentPose = (follower != null) ? follower.getPose() : null;
                    if (currentPose == null) {
                        return true;
                    }
                    Pose headingAlignPose = new Pose(
                            currentPose.getX() + 0.01,
                            currentPose.getY(),
                            getRow4PreShootHeadingRad()
                    );
                    followPath(pathLibrary.buildLinear(currentPose, headingAlignPose), ROW4_COMPLETE_PICKUP_POWER);
                    row4PreShootHeadingAlignStarted = true;
                    row4CompletePickupIdleSeen = false;
                    row4CompletePickupHoldTimer.reset();
                    return false;
                }
                return row4CompletePickupHoldTimer.seconds() >= ROW4_COMPLETE_PICKUP_HOLD_SECONDS;
            }
            row4CompletePickupIdleSeen = false;
            return false;
        }
        return hasReachedPickupBallTarget() || followerIdle() || row4PickupTimedOut();
    }

    protected double getRow4PreShootHeadingRad() {
        if (alliance == Alliance.BLUE) {
            return Math.toRadians(ROW4_PRE_SHOOT_HEADING_DEG_BLUE);
        }
        return Math.toRadians(180.0 - ROW4_PRE_SHOOT_HEADING_DEG_BLUE);
    }

    protected boolean goToPickupAdvanceReady() {
        if (stateTimedOut()) {
            return true;
        }
        if (followerIdle()) {
            if (!goToPickupIdleSeen) {
                goToPickupIdleSeen = true;
                goToPickupIdleTimer.reset();
            }
            if (currentAbsoluteRow == 4) {
                if (!row4IntermediatePickupStarted) {
                    if (goToPickupIdleTimer.seconds() < ROW4_GO_TO_PICKUP_HOLD_SECONDS) {
                        return false;
                    }
                    Pose currentPose = (follower != null) ? follower.getPose() : null;
                    PathChain intermediatePath = pathLibrary.buildLinear(
                            currentPose,
                            poses.getRow4IntermediatePickup(alliance)
                    );
                    followPath(intermediatePath, ROW4_INTERMEDIATE_PICKUP_POWER);
                    row4IntermediatePickupStarted = true;
                    // Keep this sub-step at the requested 75% power.
                    goToPickupSlowdownApplied = true;
                    goToPickupIdleSeen = false;
                    goToPickupIdleTimer.reset();
                    return false;
                }
                return goToPickupIdleTimer.seconds() >= ROW4_GO_TO_PICKUP_HOLD_SECONDS;
            }
            return goToPickupIdleTimer.seconds() >= GO_TO_PICKUP_IDLE_HOLD_SECONDS
                    && isHeadingSettledForGoToPickup();
        }
        goToPickupIdleSeen = false;
        return false;
    }

    protected boolean backRowGoToPickupAdvanceReady() {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            if (!closeLoopGoToPickupPart2Started) {
                boolean startPart2 = followerIdle()
                        || stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS;
                if (!startPart2) {
                    return false;
                }
                Pose currentPose = (follower != null) ? follower.getPose() : null;
                PathChain part2Path = pathLibrary.closeLoopPickupPart2(currentPose, alliance);
                followPath(part2Path, CLOSE_LOOP_PICKUP_PART2_POWER);
                closeLoopGoToPickupPart2Started = true;
                closeLoopGoToPickupIdleSeen = false;
                closeLoopGoToPickupIdleTimer.reset();
                resetStateTimer();
                return false;
            }
            if (stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS) {
                return true;
            }
            if (followerIdle()) {
                if (!closeLoopGoToPickupIdleSeen) {
                    closeLoopGoToPickupIdleSeen = true;
                    closeLoopGoToPickupIdleTimer.reset();
                }
                return closeLoopGoToPickupIdleTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_IDLE_DELAY_SECONDS;
            }
            closeLoopGoToPickupIdleSeen = false;
            return stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS;
        }
        if (stateTimer.seconds() >= BACKROW_PICKUP_TIMEOUT_SECONDS) {
            return true;
        }
        if (!followerIdle()) {
            return false;
        }
        if (!farBackrowRetreatStarted) {
            Pose retreatTarget = offsetFarBackrowPickupPose(FAR_BACKROW_REPOSITION_BACK_DELTA_X);
            followPath(pathLibrary.buildLinear(follower.getPose(), retreatTarget), FAR_BACKROW_REPOSITION_POWER);
            farBackrowRetreatStarted = true;
            farBackrowForwardStarted = false;
            // Disable smooth-end rebuild after starting reposition legs.
            backRowGoToPickupSlowdownApplied = true;
            return false;
        }
        if (!farBackrowForwardStarted) {
            Pose forwardTarget = offsetFarBackrowPickupPose(-FAR_BACKROW_REPOSITION_FORWARD_DELTA_X);
            followPath(pathLibrary.buildLinear(follower.getPose(), forwardTarget), FAR_BACKROW_REPOSITION_POWER);
            farBackrowForwardStarted = true;
            return false;
        }
        return followerIdle() || stateTimer.seconds() >= BACKROW_PICKUP_TIMEOUT_SECONDS;
    }

    protected Pose offsetFarBackrowPickupPose(double blueDeltaX) {
        if (follower == null || follower.getPose() == null) {
            return null;
        }
        Pose currentPose = follower.getPose();
        double signedDeltaX = (alliance == Alliance.BLUE) ? blueDeltaX : -blueDeltaX;
        return new Pose(
                currentPose.getX() + signedDeltaX,
                currentPose.getY(),
                currentPose.getHeading()
        );
    }

    protected boolean backRowCompletePickupAdvanceReady() {
        if (stateTimedOut()) {
            return true;
        }
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return followerIdle() || stateTimer.seconds() >= CLOSE_LOOP_COMPLETE_PICKUP_TIMEOUT_SECONDS;
        }
        return followerIdle() || backRowPickupTimedOut();
    }

    protected boolean row4PickupTimedOut() {
        if (!(activeState == AutoStates.COMPLETE_PICKUP && currentAbsoluteRow == 4)) {
            return false;
        }
        return stateTimer.seconds() >= ROW4_PICKUP_TIMEOUT_SECONDS;
    }

    protected boolean backRowPickupTimedOut() {
        if (activeState != AutoStates.BACKROW_LOOP_COMPLETE_PICKUP
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
        if (range == Range.CLOSE_RANGE && preloadComplete) {
            if (shouldStartNextCycle()) {
                return AutoStates.GO_TO_PICKUP;
            }
            if (shouldEnterCloseLoop()) {
                return AutoStates.CLOSE_LOOP_GO_TO_PICKUP;
            }
            if (shouldEnterFarBackRowLoop()) {
                return AutoStates.BACKROW_LOOP_GO_TO_PICKUP;
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
        return acquireMotifReturnState == AutoStates.BACKROW_LOOP_GO_TO_PICKUP;
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

    protected boolean isRow1PickupAfterPreload() {
        boolean pickupState = activeState == AutoStates.GO_TO_PICKUP
                || activeState == AutoStates.COMPLETE_PICKUP;
        return pickupState && preloadComplete && rowsCompleted == 0 && currentAbsoluteRow == 1;
    }

    protected boolean isFirstCompletePickupAfterPreload() {
        return activeState == AutoStates.COMPLETE_PICKUP
                && preloadComplete
                && rowsCompleted == 0;
    }

    protected boolean shouldAimObeliskDuringRow1Pickup() {
        return isFirstCompletePickupAfterPreload();
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
        if (shootTimedOut() || stateTimedOut()) {
            return true;
        }
        return shootActionComplete() && allShotBallsCleared();
    }

    protected boolean allShotBallsCleared() {
        return skipCurrentShot || getLoadedBallCount() <= 0;
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
            } else {
                return false;
            }
        }
        StateMachine activeShootMachine = getAutoShootMachine();
        if (activeShootMachine == null) {
            return true;
        }
        return activeShootMachine.getState() == Robot.SortedShootAllStates.INIT
                && !robot.initSortedShootAllMachine;

        //TODO get a boolean from shooter subsystem
    }

    private void armAutoShootSequence() {
        // Auto always uses sorted shooting order.
        robot.useSorting = true;
        robot.initShootAllMachine = false;
        robot.initSortedShootAllMachine = true;
    }

    protected void maybeStartShootAtPathProgress() {
        if (shootStartedInGoToShoot) {
            return;
        }
        // Shoot-while-moving is only for close-range autos.
        if (range != Range.CLOSE_RANGE) {
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
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }
        double pathT = follower.getCurrentPath().getClosestPointTValue();
        double requiredProgress = preloadComplete
                ? SHOOT_START_PATH_PROGRESS
                : PRELOAD_SHOOT_START_PATH_PROGRESS;
        if (!Double.isFinite(pathT) || pathT < requiredProgress) {
            return;
        }
        // Start shooting at 75% path progress while continuing to finish the path.
        robot.forceShootAllThreeOnNextStart = !preloadComplete;
        armAutoShootSequence();
        shootSequenceStarted = true;
        shootStartedInGoToShoot = true;
    }

    private StateMachine getAutoShootMachine() {
        return sortingShootAllMachine;
    }

    protected boolean shouldStartShootSequence() {
        return hasCompleteShootReadyConditions()
                || completeShootReadyTimer.seconds() >= COMPLETE_SHOOT_READY_TIMEOUT_SECONDS;
    }

    protected boolean hasCompleteShootReadyConditions() {
        if (robot == null || robot.outtake == null || robot.outtake.shooter == null
                || robot.outtake.turret == null || robot.outtake.vision == null) {
            return false;
        }
        boolean shooterAtRpm = robot.outtake.shooter.isAtRPM();
        boolean requiredTagVisible = robot.outtake.vision.hasRequiredTarget();
        boolean turretAligned = robot.outtake.isVisionOnTarget(
                robot.outtake.vision,
                COMPLETE_SHOOT_TURRET_TOLERANCE_DEG
        );
        boolean robotSettled = isRobotMotionSettledForShot();
        return shooterAtRpm && requiredTagVisible && turretAligned && robotSettled;
    }

    protected boolean isRobotMotionSettledForShot() {
        if (follower == null) {
            return false;
        }

        if (follower.getVelocity() == null) {
            return false;
        }

        double translationalSpeedInS = Math.abs(follower.getVelocity().getMagnitude());
        double angularSpeedDegS = Math.abs(Math.toDegrees(follower.getAngularVelocity()));
        if (Double.isNaN(translationalSpeedInS) || Double.isInfinite(translationalSpeedInS)
                || Double.isNaN(angularSpeedDegS) || Double.isInfinite(angularSpeedDegS)) {
            return false;
        }
        return translationalSpeedInS <= SHOOT_SETTLE_MAX_TRANSLATIONAL_SPEED_IN_S
                && angularSpeedDegS <= SHOOT_SETTLE_MAX_ANGULAR_SPEED_DEG_S;
    }

    protected void setActiveState(AutoStates state) {
        activeState = state;
    }

    protected void resetStateTimer() {
        stateTimer.reset();
    }

    /** Waits 1s after the release path finishes before advancing. */
    protected boolean releaseWaitDone() {
        return releaseWaiter.isDone(followerIdle(), RELEASE_TIMEOUT_SECONDS);
    }

    protected boolean backRowLoopShootComplete() {
        return shootAdvanceReady();
    }

    protected boolean shouldExitBackRowLoop() {
        if (!backRowLoopShootComplete()) {
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
        if (backRowLoopCyclesTarget <= 0) {
            return true;
        }
        return (backRowLoopCyclesCompleted + 1) < backRowLoopCyclesTarget;
    }

    protected Pose getBackRowLoopScorePoseForCurrentShot() {
        return getScorePoseForCurrentShot();
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
            case BACKROW_LOOP_GO_TO_PICKUP:
                return "Back-row loop: go to far pickup zone";
            case BACKROW_LOOP_COMPLETE_PICKUP:
                return "Back-row loop: complete pickup";
            case BACKROW_LOOP_GO_TO_SHOOT:
                return "Back-row loop: go to shoot spot";
            case BACKROW_LOOP_COMPLETE_SHOOT:
                return "Back-row loop: shooting";
            case CLOSE_LOOP_GO_TO_PICKUP:
                return "Close loop: go to pickup";
            case CLOSE_LOOP_COMPLETE_PICKUP:
                return "Close loop: complete pickup";
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

