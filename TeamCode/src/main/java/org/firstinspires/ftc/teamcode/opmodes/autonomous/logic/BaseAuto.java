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
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoShootMachine;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoTurretAim;
import org.firstinspires.ftc.teamcode.config.autoUtil.ReleaseWaiter;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

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
    private PathChain leavePath;
    private PathChain releaseGoToPath;
    private PathChain releaseCompletePath;
    private double intakeSpeed = 0.25;
    private final AutoIntakeSpeed intakeSpeedModel = new AutoIntakeSpeed(
            -0.02, 0.47, 0.18, 0.24, -0.01, 0.01);

    // ===== Constants =====
    private static final double SHOOT_ACTION_SECONDS = 10.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 1.0;
    private static final double STATE_TIMEOUT_SECONDS = 5.0; // fallback: force state advance after this time
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
        GO_TO_SCORE,
        GO_TO_RELEASE,
        COMPLETE_RELEASE,
        LEAVE
    }

    // ===== State Machine =====
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;
    private AutoShootMachine shootMachine;
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
    private ReleaseWaiter releaseWaiter = new ReleaseWaiter(RELEASE_IDLE_SECONDS);

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

        shootMachine = new AutoShootMachine(robot, clutchDownTime, clutchDownFarTime, spinTime, spinUpTimeout);
        turretAim = new AutoTurretAim(robot, poses, alliance, range, telemetry);
        motifTracker = new AutoMotifTracker(robot, alliance, range, MOTIF_ACQUIRE_TIMEOUT);

        autoMachine = buildAutoMachine();

        intakeSpeed = intakeSpeedModel.compute(robot.getVoltage(), alliance, range);
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
        shootMachine.start();
    }

    @Override
    public void loop() {
        follower.update();
        turretAim.updateAim(activeState, preloadComplete);

        robot.update();
        autoMachine.update();
        shootMachine.update();

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
                .transition(this::motifAcquiredOrTimedOut, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(() -> shouldSkipShootPhase() || stateTimedOut(), AutoStates.LEAVE)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && !shouldStartNextCycle() && shouldEnterBackRowLoop(), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && !shouldStartNextCycle() && !shouldEnterBackRowLoop(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .onExit(this::onExitCompletePickup)
                .transition(() -> shouldReleaseAfterPickup() && (followerIdle() || stateTimedOut()), AutoStates.GO_TO_RELEASE)
                .transition(() -> !shouldReleaseAfterPickup() && (followerIdle() || stateTimedOut()), AutoStates.GO_TO_SHOOT) //Could add second condition of intake finished

                .state(AutoStates.GO_TO_RELEASE)
                .onEnter(this::onEnterGoToRelease)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_RELEASE)

                .state(AutoStates.COMPLETE_RELEASE)
                .onEnter(this::onEnterCompleteRelease)
                .transition(this::releaseWaitDone, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterBackRowLoopGoToPickup)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.BACKROW_LOOP_GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterBackRowLoopGoToShoot)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.BACKROW_LOOP_COMPLETE_SHOOT)

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
                .transition(this::motifAcquiredOrTimedOut, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(() -> shouldSkipShootPhase() || stateTimedOut(), AutoStates.LEAVE)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle(), AutoStates.LEAVE)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    // ===== State Machine Callbacks =====
    protected void onEnterAcquireMotif() {
        setActiveState(AutoStates.ACQUIRE_MOTIF);

        // TODO: reset motif acquisition command state.
        // TODO: start turret/vision command to acquire motif.
    }

    protected boolean motifAcquiredOrTimedOut() {
        return motifTracker.hasMotifOrTimedOut();
    }

    protected void onExitAcquireMotif() {
        // TODO: finalize motif acquisition and persist selected motif.
    }

    protected void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);

        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }

        // TODO: start shooter/intake prep command while driving to score.
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);
        // TODO: run shoot command sequence and mark completion from subsystem signal.
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

        // TODO: intake subsystem command can be stopped here once transfer is complete.
        // TODO: shooter subsystem spin-up / turret-lock command should begin before arrival.
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterBackRowLoopCompleteShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT);

        resetStateTimer();
        shootTimer.reset();

        // TODO: shooter subsystem command to fire loop shot goes here.
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
        return pathLibrary.goToPickup(currentPose, alliance, currentAbsoluteRow);
    }

    protected PathChain buildPickupPath(Pose currentPose) {
        return pathLibrary.pickup(currentPose, alliance, currentAbsoluteRow);
    }

    protected PathChain buildGoToScorePath(Pose currentPose) {
        return pathLibrary.goToScore(currentPose, getScorePoseForCurrentShot());
    }

    protected PathChain buildFarPickupZonePath(Pose currentPose) {
        return pathLibrary.farPickupZone(currentPose, alliance);
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

    protected boolean shouldEnterBackRowLoop() {
        return backRowLoopEnabled;
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
        return shootMachine.isShootingComplete();

        //TODO get a boolean from shooter subsystem
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
        // Placeholder completion signal for loop shot until subsystem signal is wired.
        return shootTimedOut() || stateTimedOut();
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
