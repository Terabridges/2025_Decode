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

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPathLibrary;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoRoutePlanner;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

/**
 * Path-only autonomous base for route validation.
 * Does not initialize or command robot subsystems (turret/shooter/intake/etc).
 */
@PsiKitAutoLog(rlogPort = 5802)
public abstract class BaseAutoPathTesting extends OpMode {

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

    private static final double STATE_TIMEOUT_SECONDS = 5.0;
    private static final double PICKUP_POWER = 0.25;
    private static final double RELEASE_COMPLETE_POWER = 0.35;

    private final Alliance alliance;
    private Range range;
    private boolean releaseAfterClosePickup;
    private boolean shootPreload;
    private boolean allowPickupCycles;
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

    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;

    private int[] rowSequence = new int[0];
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentAbsoluteRow = 1;
    private boolean preloadComplete = false;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private boolean previousGamepad1A = false;
    private boolean gamepad1APressedEdge = false;

    protected BaseAutoPathTesting(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
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

        autoMachine = buildPathTestingStateMachine();

        rowsToRun = rowSequence.length;
        rowsCompleted = 0;
        currentAbsoluteRow = (rowsToRun > 0) ? rowSequence[0] : routePlanner.getStartingAbsoluteRow();
        preloadComplete = false;
        backRowLoopCyclesCompleted = 0;

        FollowerManager.initFollower(hardwareMap, startPose);
        stateTimer.reset();
        previousGamepad1A = false;
        gamepad1APressedEdge = false;
    }

    @Override
    public void init_loop() {
        telemetryM.debug("PathTest: " + this.getClass().getSimpleName() + " | State: " + activeState);
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        autoMachine.start();
    }

    @Override
    public void loop() {
        gamepad1APressedEdge = gamepad1 != null && gamepad1.a && !previousGamepad1A;
        previousGamepad1A = gamepad1 != null && gamepad1.a;

        follower.update();
        autoMachine.update();

        Pose pose = (follower != null) ? follower.getPose() : null;
        telemetryM.debug("PathTest: " + this.getClass().getSimpleName() + " | State: " + activeState);
        telemetryM.debug("PathTest: press gamepad1 A to advance");
        if (pose != null) {
            telemetryM.debug(String.format("Follower: x=%.2f y=%.2f h=%.1fdeg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        } else {
            telemetryM.debug("Follower: pose unavailable");
        }
        telemetryM.debug("PathTest rows: toRun=" + rowsToRun + " completed=" + rowsCompleted
                + " current=" + currentAbsoluteRow + " preload=" + preloadComplete);
        telemetryM.debug("PathTest branches: pickup=" + shouldGoToPickupAfterShot()
                + " backrow=" + shouldGoToBackRowLoopAfterShot()
                + " leave=" + shouldLeaveAfterShot());
        telemetryM.update(telemetry);
        telemetry.update();

        drawCurrentAndHistory();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    protected abstract AutoSpec getSpec();

    protected final StateMachine buildPathTestingStateMachine() {
        return new StateMachineBuilder()
                .state(AutoStates.ACQUIRE_MOTIF)
                .onEnter(() -> setActiveState(AutoStates.ACQUIRE_MOTIF))
                .transition(() -> advanceApproved(true), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(() -> advanceApproved(shouldSkipShootPhase()), AutoStates.LEAVE)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(() -> setActiveState(AutoStates.COMPLETE_SHOOT))
                .onExit(this::onExitCompleteShoot)
                .transition(() -> advanceApproved(followerIdle() && shouldGoToPickupAfterShot()), AutoStates.GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldGoToBackRowLoopAfterShot()), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldLeaveAfterShot()), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .transition(() -> advanceApproved(shouldReleaseAfterPickup() && followerIdle()), AutoStates.GO_TO_RELEASE)
                .transition(() -> advanceApproved(!shouldReleaseAfterPickup() && followerIdle()), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_RELEASE)
                .onEnter(this::onEnterGoToRelease)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.COMPLETE_RELEASE)

                .state(AutoStates.COMPLETE_RELEASE)
                .onEnter(this::onEnterCompleteRelease)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterBackRowLoopGoToPickup)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.BACKROW_LOOP_GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterBackRowLoopGoToShoot)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.BACKROW_LOOP_COMPLETE_SHOOT)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT)
                .onEnter(this::onEnterBackRowLoopCompleteShoot)
                .onExit(this::onExitBackRowLoopCompleteShoot)
                .transition(() -> advanceApproved(shouldExitBackRowLoop()), AutoStates.LEAVE)
                .transition(() -> advanceApproved(shouldContinueBackRowLoop()), AutoStates.BACKROW_LOOP_GO_TO_PICKUP)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    protected void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);
        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0;
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, rowsToRun);
        }
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
        followPath(pickupPath, PICKUP_POWER);
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
        buildPath(PathRequest.COMPLETE_RELEASE);
        followPath(releaseCompletePath, RELEASE_COMPLETE_POWER);
    }

    protected void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);
        resetStateTimer();
        lastScoreRangeUsed = getLeaveRangeForLastShot();
        buildPath(PathRequest.LEAVE);
        followPath(leavePath);
    }

    protected void onEnterBackRowLoopGoToPickup() {
        setActiveState(AutoStates.BACKROW_LOOP_GO_TO_PICKUP);
        resetStateTimer();
        buildPath(PathRequest.GO_TO_FAR_PICKUP_ZONE);
        followPath(backRowLoopPickupPath);
    }

    protected void onEnterBackRowLoopGoToShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_GO_TO_SHOOT);
        resetStateTimer();
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    protected void onEnterBackRowLoopCompleteShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT);
        resetStateTimer();
    }

    protected void onExitBackRowLoopCompleteShoot() {
        backRowLoopCyclesCompleted++;
    }

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
                goToPickupPath = pathLibrary.goToPickup(currentPose, alliance, currentAbsoluteRow);
                break;
            case COMPLETE_PICKUP:
                pickupPath = pathLibrary.pickup(currentPose, alliance, currentAbsoluteRow);
                break;
            case GO_TO_FAR_PICKUP_ZONE:
                backRowLoopPickupPath = pathLibrary.farPickupZone(currentPose, alliance);
                break;
            case GO_TO_SCORE:
                lastScoreRangeUsed = getScoreRangeForCurrentShot();
                goToScorePath = pathLibrary.goToScore(currentPose, getScorePoseForCurrentShot());
                break;
            case GO_TO_RELEASE:
                releaseGoToPath = pathLibrary.releaseGoTo(currentPose, alliance, range);
                break;
            case COMPLETE_RELEASE:
                releaseCompletePath = pathLibrary.releaseComplete(currentPose, alliance, range);
                break;
            case LEAVE:
                leavePath = pathLibrary.leave(currentPose, alliance, lastScoreRangeUsed);
                break;
        }
    }

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
            Pose closeIntakeHeadingPose = poses.getPickupStart(alliance, 1);
            return new Pose(base.getX(), base.getY(), closeIntakeHeadingPose.getHeading());
        }
        return base;
    }

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

    protected boolean shouldShootPreload() {
        return shootPreload;
    }

    protected boolean shouldReleaseAfterPickup() {
        boolean justFinishedFirstRow = preloadComplete && rowsCompleted == 0;
        return allowPickupCycles && justFinishedFirstRow && releaseAfterClosePickup;
    }

    protected boolean shouldStartNextCycle() {
        if (!preloadComplete) {
            return rowsToRun > 0;
        }
        int nextCount = rowsCompleted + 1;
        return nextCount < rowsToRun;
    }

    protected boolean shouldSkipShootPhase() {
        return !preloadComplete && !shouldShootPreload();
    }

    protected boolean shouldEnterBackRowLoop() {
        return backRowLoopEnabled;
    }

    protected boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    protected void setActiveState(AutoStates state) {
        activeState = state;
    }

    protected void resetStateTimer() {
        stateTimer.reset();
    }

    protected boolean shouldExitBackRowLoop() {
        if (backRowLoopCyclesTarget <= 0) {
            return false;
        }
        return (backRowLoopCyclesCompleted + 1) >= backRowLoopCyclesTarget;
    }

    protected boolean shouldContinueBackRowLoop() {
        if (backRowLoopCyclesTarget <= 0) {
            return true;
        }
        return (backRowLoopCyclesCompleted + 1) < backRowLoopCyclesTarget;
    }

    protected boolean advanceApproved(boolean conditionReady) {
        return conditionReady && gamepad1APressedEdge;
    }

    /**
     * Path-testing branch decisions are explicit to avoid ambiguity around preload transitions.
     */
    protected boolean shouldGoToPickupAfterShot() {
        if (!preloadComplete) {
            return rowsToRun > 0;
        }
        return shouldStartNextCycle();
    }

    protected boolean shouldGoToBackRowLoopAfterShot() {
        if (!preloadComplete) {
            return false;
        }
        return !shouldStartNextCycle() && shouldEnterBackRowLoop();
    }

    protected boolean shouldLeaveAfterShot() {
        return !shouldGoToPickupAfterShot() && !shouldGoToBackRowLoopAfterShot();
    }
}
