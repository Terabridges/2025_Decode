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
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

//@PsiKitAutoLog(rlogPort = 5802)
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
    private PathChain releaseGoToPath;
    private PathChain releaseCompletePath;
    private double intakeSpeed = 0.25;

//    private final AutoIntakeSpeed intakeSpeedModel = new AutoIntakeSpeed(
//            -0.02, 0.47, 0.18, 0.24, -0.01, 0.01);

    // ===== Constants =====
    private static final double SHOOT_ACTION_SECONDS = 5.0;
    private static final double COMPLETE_SHOOT_READY_TIMEOUT_SECONDS = 2.0;
    private static final double COMPLETE_SHOOT_TURRET_TOLERANCE_DEG = 2.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 1.5;
    private static final double MOTIF_ACQUIRE_AIM_WINDOW_SECONDS = 0.5;
    private static final double STATE_TIMEOUT_SECONDS = 4.0; // fallback: force state advance after this time
    private static final int PICKUP_TARGET_BALL_COUNT = 3;
    private static final int TAG_BLUE = 20;
    private static final int TAG_RED = 24;
    private static final double RELEASE_IDLE_SECONDS = 1.0;
    private static final double RELEASE_TIMEOUT_SECONDS = 2.25;

    private final Alliance alliance;
    private Range range;
    private boolean releaseAfterClosePickup;
    private boolean shootPreload;
    private boolean allowPickupCycles;
    @SuppressWarnings("unused")
    private boolean backRowLoopEnabled;
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
        GO_TO_RELEASE,
        COMPLETE_RELEASE,
        LEAVE
    }

    // ===== State Machine =====
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;
    private StateMachine shootAllMachine;
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
    private ReleaseWaiter releaseWaiter = new ReleaseWaiter(RELEASE_IDLE_SECONDS);
    private AutoStates acquireMotifReturnState = AutoStates.GO_TO_SHOOT;
    private boolean motifResolvedThisAcquire = false;
    private boolean shootSequenceStarted = false;
    private boolean skipCurrentShot = false;

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
        backRowLoopCyclesTarget = spec.backRowLoopCycles;
        rowSequence = spec.rowSequence;
        allowPickupCycles = rowSequence.length > 0;
        routePlanner = new AutoRoutePlanner(range);
        lastScoreRangeUsed = range;
        startPose = poses.findStartPose(alliance, range);

        robot.other.drive.manualDrive = false;

        shootAllMachine = robot.getShootAllMachine();
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
    }

    @Override
    public void loop() {
        follower.update();

        autoMachine.update();
        turretAim.updateAim(activeState, preloadComplete);
        robot.update();
        if (shootAllMachine != null) {
            shootAllMachine.update();
        }

        telemetryM.debug("Auto: " + this.getClass().getSimpleName() + " | State: " + activeState);
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
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToBackrowPickupAfterAcquire(), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .transition(() -> motifAcquiredOrTimedOut() && shouldReturnToLeaveAfterAcquire(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(this::shouldSkipShootPhase, AutoStates.LEAVE)
                .transition(this::followerIdle, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && shouldAcquireMotifAfterPreloadShot(), AutoStates.ACQUIRE_MOTIF)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && !shouldStartNextCycle() && shouldEnterBackRowLoop(), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && !shouldStartNextCycle() && !shouldEnterBackRowLoop(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
                .transition(this::followerIdle, AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .onExit(this::onExitCompletePickup)
                .transition(() -> shouldReleaseAfterPickup() && pickupAdvanceReady(), AutoStates.GO_TO_RELEASE)
                .transition(() -> !shouldReleaseAfterPickup() && pickupAdvanceReady(), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_RELEASE)
                .onEnter(this::onEnterGoToRelease)
                .transition(this::followerIdle, AutoStates.COMPLETE_RELEASE)

                .state(AutoStates.COMPLETE_RELEASE)
                .onEnter(this::onEnterCompleteRelease)
                .transition(this::releaseWaitDone, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterBackRowLoopGoToPickup)
                .transition(this::followerIdle, AutoStates.BACKROW_LOOP_COMPLETE_PICKUP)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP)
                .onEnter(this::onEnterBackRowLoopCompletePickup)
                .transition(this::followerIdle, AutoStates.BACKROW_LOOP_GO_TO_SHOOT)

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
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && shouldAcquireMotifAfterPreloadShot(), AutoStates.ACQUIRE_MOTIF)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle(), AutoStates.LEAVE)

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

        prepareForShootWhileDriving();
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);
        resetStateTimer();
        shootTimer.reset();
        completeShootReadyTimer.reset();
        shootSequenceStarted = false;
        // Do not trust startup ball count for preload; only skip empty shots after preload.
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;
    }

    protected void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0; // start counting rows after preload
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, rowsToRun);
        }
        // TODO: clear shoot command completion state.
    }

    protected void onEnterGoToPickup() {
        setActiveState(AutoStates.GO_TO_PICKUP);

        resetStateTimer();

        refreshCurrentAbsoluteRow();
        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(goToPickupPath);
    }

    protected void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);
        resetStateTimer();
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        buildPath(PathRequest.COMPLETE_PICKUP);
        // TODO: run intake command while completing pickup path.
        followPath(pickupPath, intakeSpeed);
    }

    protected void onExitCompletePickup() {
        // TODO: stop intake command after pickup completes.
    }

    protected void onEnterGoToRelease() {
        setActiveState(AutoStates.GO_TO_RELEASE);

        resetStateTimer();

        buildPath(PathRequest.GO_TO_RELEASE);
        followPath(releaseGoToPath);
    }

    protected void onEnterCompleteRelease() {
        setActiveState(AutoStates.COMPLETE_RELEASE);

        resetStateTimer();
        releaseWaiter.reset();

        buildPath(PathRequest.COMPLETE_RELEASE);
        followPath(releaseCompletePath, 0.35);
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

        // TODO: intake subsystem command should keep intake running for this full transfer leg.
        buildPath(PathRequest.GO_TO_FAR_PICKUP_ZONE);
        followPath(backRowLoopPickupPath);
    }

    protected void onEnterBackRowLoopGoToShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_GO_TO_SHOOT);

        resetStateTimer();

        // Stop intake transfer and pre-stage shot while driving to score.
        prepareForShootWhileDriving();
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterBackRowLoopCompletePickup() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP);

        resetStateTimer();
        robot.intake.spinner.setMegaSpinIn();
        robot.intake.clutch.setClutchUp();

        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP);
        followPath(backRowLoopCompletePickupPath, intakeSpeed);
    }

    protected void onEnterBackRowLoopCompleteShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT);

        resetStateTimer();
        shootTimer.reset();
        completeShootReadyTimer.reset();
        shootSequenceStarted = false;
        skipCurrentShot = preloadComplete && getLoadedBallCount() <= 0;

        // Run the same shoot sequence logic used by COMPLETE_SHOOT for back-row loop shots.
    }

    protected void onExitBackRowLoopCompleteShoot() {
        backRowLoopCyclesCompleted++;
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
                backRowLoopPickupPath = buildFarPickupZonePath(currentPose);
                break;
            case BACKROW_COMPLETE_PICKUP:
                backRowLoopCompletePickupPath = buildBackRowLoopCompletePickupPath(currentPose);
                break;
            case GO_TO_SCORE:
                lastScoreRangeUsed = getScoreRangeForCurrentShot();
                goToScorePath = buildGoToScorePath(currentPose);
                break;
            case GO_TO_RELEASE:
                releaseGoToPath = buildReleaseGoToPath(currentPose);
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
        return pathLibrary.goToPickup(currentPose, alliance, range, currentAbsoluteRow);
    }

    protected PathChain buildPickupPath(Pose currentPose) {
        return pathLibrary.pickup(currentPose, alliance, range, currentAbsoluteRow);
    }

    protected PathChain buildGoToScorePath(Pose currentPose) {
        return pathLibrary.goToScore(currentPose, getScorePoseForCurrentShot());
    }

    protected PathChain buildFarPickupZonePath(Pose currentPose) {
        return pathLibrary.farPickupZone(currentPose, alliance);
    }

    protected PathChain buildBackRowLoopCompletePickupPath(Pose currentPose) {
        // Back-row loop always intakes from absolute row 4.
        return pathLibrary.pickup(currentPose, alliance, Range.LONG_RANGE, 4);
    }

    protected PathChain buildReleaseGoToPath(Pose currentPose) {
        return pathLibrary.releaseGoTo(currentPose, alliance, range);
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
        Pose base = poses.getScore(alliance, getScoreRangeForCurrentShot());
        if (getScoreRangeForCurrentShot() == Range.CLOSE_RANGE && preloadComplete) {
            // Close-range cycle shots should face the same direction as intake heading.
            Pose closeIntakeHeadingPose = poses.getPickupStart(alliance, 1);
            return new Pose(base.getX(), base.getY(), closeIntakeHeadingPose.getHeading());
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

    protected boolean pickupAdvanceReady() {
        return hasReachedPickupBallTarget() || followerIdle();
    }

    protected boolean hasReachedPickupBallTarget() {
        return getLoadedBallCount() >= PICKUP_TARGET_BALL_COUNT;
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
            if (shouldEnterBackRowLoop()) {
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

    // ===== State/Timer Helpers =====
    protected boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    protected boolean shootTimedOut() {
        return shootTimer.seconds() >= SHOOT_ACTION_SECONDS;
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
                robot.initShootAllMachine = true;
                shootSequenceStarted = true;
            } else {
                return false;
            }
        }
        if (shootAllMachine == null) {
            return true;
        }
        return shootAllMachine.getState() == Robot.ShootAllStates.INIT && !robot.initShootAllMachine;

        //TODO get a boolean from shooter subsystem
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
        boolean turretAligned = robot.outtake.turret.isVisionOnTarget(
                robot.outtake.vision,
                COMPLETE_SHOOT_TURRET_TOLERANCE_DEG
        );
        return shooterAtRpm && requiredTagVisible && turretAligned;
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
        return shootActionComplete() || shootTimedOut() || stateTimedOut();
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
            case LEAVE:
                return "Parking / leave path";
            default:
                return "Idle";
        }
    }

}
