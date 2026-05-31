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
/**
 * Path-only autonomous base for route validation.
 * Does not initialize or command robot subsystems (turret/shooter/intake/etc).
 */
public abstract class BaseAutoPathTesting extends OpMode {

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

    private static final double STATE_TIMEOUT_SECONDS = 5.0;
    private static final double PATH_ADVANCE_PROGRESS = 0.90;
    private static final double GO_TO_SHOOT_PATH_ADVANCE_PROGRESS = 0.85;
    private static final double GO_TO_PICKUP_PATH_ADVANCE_PROGRESS = 0.75;
    private static final double RELEASE_IDLE_SECONDS = 0.0;
    private static final double RELEASE_TIMEOUT_SECONDS = 1.5;
    private static final double CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS = 1.05;
    private static final double CLOSE_LOOP_GO_TO_PICKUP_IDLE_DELAY_SECONDS = 0.0;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_TIMEOUT_SECONDS = 1.5;
    private static final double PICKUP_POWER = 0.40;
    private static final double FAR_PICKUP_ZONE_POWER = 1.0;
    private static final double CLOSE_LOOP_PICKUP_ZONE_POWER = 1.0;
    private static final double CLOSE_LOOP_PICKUP_PART2_POWER = 1.0;
    private static final double CLOSE_LOOP_COMPLETE_PICKUP_FIRST_HALF_POWER = 1.0;
    private static final double BACKROW_COMPLETE_PICKUP_POWER = 1.0;
    private static final double ROW4_COMPLETE_PICKUP_POWER = 1.0;
    private static final double ROW3_COMPLETE_PICKUP_POWER = 0.40;
    private static final double RELEASE_COMPLETE_POWER = 1.0;
    private static final double RED_CLOSE_PRELOAD_GO_TO_SHOOT_POWER = 1.0;

    private final Alliance alliance;
    private Range range;
    private boolean releaseAfterClosePickup;
    private boolean shootPreload;
    private boolean allowPickupCycles;
    private boolean backRowLoopEnabled;
    private boolean closeLoopEnabled;
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

    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;

    private int[] rowSequence = new int[0];
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentAbsoluteRow = 1;
    private boolean preloadComplete = false;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime closeLoopGoToPickupIdleTimer = new ElapsedTime();
    private final ElapsedTime closeLoopCompletePickupIdleTimer = new ElapsedTime();
    private final ElapsedTime backRowCompletePickupHoldTimer = new ElapsedTime();
    private boolean previousGamepad1A = false;
    private boolean gamepad1APressedEdge = false;
    private boolean closeLoopGoToPickupIdleSeen = false;
    private boolean closeLoopGoToPickupPart2Started = false;
    private boolean closeLoopCompletePickupIdleSeen = false;
    private boolean closeLoopCycleActive = false;
    private boolean backRowCompletePickupHoldSeen = false;

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
        closeLoopEnabled = spec.closeLoopEnabled;
        backRowLoopPostIntakeHoldSeconds = spec.backRowLoopPostIntakeHoldSeconds;
        backRowLoopCyclesTarget = backRowLoopEnabled && !closeLoopEnabled
                ? Math.max(2, spec.backRowLoopCycles)
                : spec.backRowLoopCycles;
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
                .transition(() -> advanceApproved(pathReadyForNextAction()), AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> advanceApproved(followerIdle() && shouldContinueActiveCloseLoop()), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldExitActiveCloseLoopToPickup()), AutoStates.GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldExitActiveCloseLoopToLeave()), AutoStates.LEAVE)
                .transition(() -> advanceApproved(followerIdle() && shouldGoToCloseLoopAfterShot()), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldGoToPickupAfterShot()), AutoStates.GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldGoToBackRowLoopAfterShot()), AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1)
                .transition(() -> advanceApproved(followerIdle() && shouldLeaveAfterShot()), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
                .transition(() -> advanceApproved(followerIdle()), AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .transition(() -> advanceApproved(shouldReleaseAfterPickup() && followerIdle()), AutoStates.COMPLETE_RELEASE)
                .transition(() -> advanceApproved(!shouldReleaseAfterPickup() && followerIdle()), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.COMPLETE_RELEASE)
                .onEnter(this::onEnterCompleteRelease)
                .transition(() -> advanceApproved(followerIdle() || releasePathTimedOut()), AutoStates.RELEASE_WAIT)

                .state(AutoStates.RELEASE_WAIT)
                .onEnter(this::onEnterReleaseWait)
                .transition(() -> advanceApproved(stateTimer.seconds() >= RELEASE_IDLE_SECONDS), AutoStates.GO_TO_SHOOT)

                .state(AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .onEnter(this::onEnterCloseLoopGoToPickup)
                .transition(() -> advanceApproved(backRowGoToPickupAdvanceReady()), AutoStates.CLOSE_LOOP_COMPLETE_PICKUP)

                .state(AutoStates.CLOSE_LOOP_COMPLETE_PICKUP)
                .onEnter(this::onEnterCloseLoopCompletePickup)
                .transition(() -> advanceApproved(backRowCompletePickupAdvanceReady()), AutoStates.CLOSE_LOOP_WAIT)

                .state(AutoStates.CLOSE_LOOP_WAIT)
                .onEnter(this::onEnterCloseLoopWait)
                .transition(() -> advanceApproved(stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_IDLE_DELAY_SECONDS), AutoStates.CLOSE_LOOP_GO_TO_SHOOT)

                .state(AutoStates.CLOSE_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterCloseLoopGoToShoot)
                .transition(() -> advanceApproved(pathReadyForNextAction()), AutoStates.CLOSE_LOOP_COMPLETE_SHOOT)

                .state(AutoStates.CLOSE_LOOP_COMPLETE_SHOOT)
                .onEnter(this::onEnterCloseLoopCompleteShoot)
                .onExit(this::onExitCloseLoopCompleteShoot)
                .transition(() -> advanceApproved(followerIdle() && shouldContinueActiveCloseLoop()), AutoStates.CLOSE_LOOP_GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldExitActiveCloseLoopToPickup()), AutoStates.GO_TO_PICKUP)
                .transition(() -> advanceApproved(followerIdle() && shouldExitActiveCloseLoopToLeave()), AutoStates.LEAVE)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1)
                .onEnter(this::onEnterBackRowLoopCompletePickup1)
                .transition(() -> advanceApproved(backRowCompletePickup1PathDone()), AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2)

                .state(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2)
                .onEnter(this::onEnterBackRowLoopCompletePickup2)
                .transition(() -> advanceApproved(backRowCompletePickup2Done()), AutoStates.BACKROW_LOOP_GO_TO_SHOOT)

                .state(AutoStates.BACKROW_LOOP_GO_TO_SHOOT)
                .onEnter(this::onEnterBackRowLoopGoToShoot)
                .onExit(this::onExitBackRowLoopCompleteShoot)
                .transition(() -> advanceApproved(shouldExitBackRowLoop()), AutoStates.LEAVE)
                .transition(() -> advanceApproved(shouldContinueBackRowLoop()), AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_1)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    protected void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);
        resetStateTimer();
        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }
        buildPath(PathRequest.GO_TO_SCORE);
        if (!preloadComplete && alliance == Alliance.RED && range == Range.CLOSE_RANGE) {
            followPath(goToScorePath, RED_CLOSE_PRELOAD_GO_TO_SHOOT_POWER);
            return;
        }
        followPath(goToScorePath);
    }

    protected void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);
        resetStateTimer();
    }

    protected void onExitCompleteShoot() {
        if (closeLoopCycleActive) {
            backRowLoopCyclesCompleted++;
            return;
        }
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
        closeLoopCycleActive = false;
        refreshCurrentAbsoluteRow();
        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(goToPickupPath);
    }

    protected void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);
        buildPath(PathRequest.COMPLETE_PICKUP);
        if (currentAbsoluteRow == 4) {
            followPath(pickupPath, ROW4_COMPLETE_PICKUP_POWER);
        } else if (currentAbsoluteRow == 3) {
            followPath(pickupPath, ROW3_COMPLETE_PICKUP_POWER);
        } else {
            followPath(pickupPath, PICKUP_POWER);
        }
    }

    protected void onEnterCompleteRelease() {
        setActiveState(AutoStates.COMPLETE_RELEASE);
        resetStateTimer();
        buildPath(PathRequest.COMPLETE_RELEASE);
        followPath(releaseCompletePath, RELEASE_COMPLETE_POWER);
    }

    protected void onEnterReleaseWait() {
        setActiveState(AutoStates.RELEASE_WAIT);
        resetStateTimer();
    }

    protected void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);
        resetStateTimer();
        closeLoopCycleActive = false;
        lastScoreRangeUsed = getLeaveRangeForLastShot();
        buildPath(PathRequest.LEAVE);
        followPath(leavePath);
    }

    protected void onEnterBackRowLoopGoToPickup() {
        setActiveState(AutoStates.CLOSE_LOOP_GO_TO_PICKUP);
        resetStateTimer();
        closeLoopGoToPickupIdleSeen = false;
        closeLoopGoToPickupIdleTimer.reset();
        closeLoopGoToPickupPart2Started = false;
        buildPath(PathRequest.GO_TO_FAR_PICKUP_ZONE);
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            followPath(backRowLoopPickupPath, CLOSE_LOOP_PICKUP_ZONE_POWER);
        } else {
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
        backRowCompletePickupHoldSeen = false;
        backRowCompletePickupHoldTimer.reset();
        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP_1);
        followPath(backRowLoopCompletePickupPath, BACKROW_COMPLETE_PICKUP_POWER);
    }

    protected void onEnterBackRowLoopCompletePickup2() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_PICKUP_2);
        resetStateTimer();
        backRowCompletePickupHoldSeen = false;
        backRowCompletePickupHoldTimer.reset();
        buildPath(PathRequest.BACKROW_COMPLETE_PICKUP_2);
        followPath(backRowLoopCompletePickupPath, BACKROW_COMPLETE_PICKUP_POWER);
    }

    protected void onEnterCloseLoopCompletePickup() {
        setActiveState(AutoStates.CLOSE_LOOP_COMPLETE_PICKUP);
        resetStateTimer();
        closeLoopCompletePickupIdleSeen = false;
        closeLoopCompletePickupIdleTimer.reset();
        Pose currentPose = (follower != null) ? follower.getPose() : null;
        PathChain part2Path = pathLibrary.closeLoopPickupPart2(currentPose, alliance);
        followPath(part2Path, CLOSE_LOOP_PICKUP_PART2_POWER);
    }

    protected void onEnterCloseLoopWait() {
        setActiveState(AutoStates.CLOSE_LOOP_WAIT);
        resetStateTimer();
    }

    protected void onEnterBackRowLoopCompleteShoot() {
        setActiveState(AutoStates.BACKROW_LOOP_COMPLETE_SHOOT);
        resetStateTimer();
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
                releaseCompletePath = pathLibrary.releaseComplete(currentPose, alliance, range);
                break;
            case LEAVE:
                leavePath = pathLibrary.leave(currentPose, alliance, lastScoreRangeUsed);
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

    protected PathChain buildPickupPath(Pose currentPose) {
        if (currentAbsoluteRow == 4) {
            return pathLibrary.row4CompletePickup(currentPose, alliance);
        }
        return pathLibrary.pickup(currentPose, alliance, range, currentAbsoluteRow);
    }

    protected boolean shouldUseCurvedRow2GoToPickup() {
        return false;
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
        Range scoreRange = getScoreRangeForCurrentShot();
        Pose base = poses.getScore(alliance, scoreRange);
        if (!preloadComplete && range == Range.LONG_RANGE && startPose != null) {
            return startPose;
        }
        if (scoreRange == Range.CLOSE_RANGE && preloadComplete && currentAbsoluteRow == 2) {
            Pose row2Pose = poses.getRow2ShootClose(alliance);
            return new Pose(row2Pose.getX(), row2Pose.getY(), Math.toRadians(0.0));
        }
        if (scoreRange == Range.CLOSE_RANGE
                && preloadComplete
                && isFinalPlannedRowShot()
                && !shouldStartNextCycle()
                && !shouldGoToCloseLoopAfterShot()) {
            return poses.getFinalShootClose(alliance);
        }
        if (scoreRange == Range.CLOSE_RANGE && preloadComplete) {
            double headingDeg = 0.0;
            if (alliance == Alliance.RED && currentAbsoluteRow == 1) {
                headingDeg = Math.toDegrees(poses.getFinalShootClose(alliance).getHeading());
            }
            return new Pose(base.getX(), base.getY(), Math.toRadians(headingDeg));
        }
        return base;
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

    protected Pose getBackRowLoopScorePoseForCurrentShot() {
        Pose scorePose = getScorePoseForCurrentShot();
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            Pose preloadPose = poses.getScore(alliance, Range.CLOSE_RANGE);
            return new Pose(preloadPose.getX(), preloadPose.getY(), scorePose.getHeading());
        }
        return scorePose;
    }

    protected boolean isGoToShootPathState() {
        return activeState == AutoStates.GO_TO_SHOOT
                || activeState == AutoStates.BACKROW_LOOP_GO_TO_SHOOT
                || activeState == AutoStates.CLOSE_LOOP_GO_TO_SHOOT;
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

    private boolean isFinalPlannedRowShot() {
        if (rowsToRun <= 0 || rowSequence.length == 0) {
            return false;
        }
        int finalIndex = Math.min(rowsToRun, rowSequence.length) - 1;
        return currentAbsoluteRow == rowSequence[finalIndex];
    }

    protected boolean shouldSkipShootPhase() {
        return !preloadComplete && !shouldShootPreload();
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

    protected boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    protected boolean backRowGoToPickupAdvanceReady() {
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            return followerIdle()
                    || stateTimer.seconds() >= CLOSE_LOOP_GO_TO_PICKUP_TIMEOUT_SECONDS;
        }
        if (stateTimedOut()) {
            return true;
        }
        return followerIdle();
    }

    protected boolean backRowCompletePickupAdvanceReady() {
        boolean ready;
        if (closeLoopEnabled && range == Range.CLOSE_RANGE) {
            ready = followerIdle()
                    || stateTimer.seconds() >= CLOSE_LOOP_COMPLETE_PICKUP_TIMEOUT_SECONDS;
        } else {
            ready = followerIdle();
        }
        return backRowCompletePickupHoldSatisfied(ready);
    }

    protected boolean backRowCompletePickup1PathDone() {
        return followerIdle() || stateTimedOut();
    }

    protected boolean backRowCompletePickup2Done() {
        return followerIdle() || stateTimedOut();
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

    protected boolean releasePathTimedOut() {
        return activeState == AutoStates.COMPLETE_RELEASE
                && stateTimer.seconds() >= RELEASE_TIMEOUT_SECONDS;
    }

    protected void setActiveState(AutoStates state) {
        activeState = state;
    }

    protected void resetStateTimer() {
        stateTimer.reset();
    }

    protected boolean shouldExitBackRowLoop() {
        if (!pathReadyForNextAction()) {
            return false;
        }
        if (backRowLoopCyclesTarget <= 0) {
            return false;
        }
        return (backRowLoopCyclesCompleted + 1) >= backRowLoopCyclesTarget;
    }

    protected boolean shouldContinueBackRowLoop() {
        if (!pathReadyForNextAction()) {
            return false;
        }
        if (backRowLoopCyclesTarget <= 0) {
            return true;
        }
        return (backRowLoopCyclesCompleted + 1) < backRowLoopCyclesTarget;
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
        return shouldStartNextCycle() && !shouldGoToCloseLoopAfterShot();
    }

    protected boolean shouldGoToBackRowLoopAfterShot() {
        if (!preloadComplete) {
            return false;
        }
        return !shouldStartNextCycle() && shouldEnterFarBackRowLoop();
    }

    protected boolean shouldGoToCloseLoopAfterShot() {
        if (!preloadComplete) {
            return false;
        }
        return !closeLoopCycleActive
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

    protected boolean hasPendingPickupRow() {
        return preloadComplete && rowsCompleted < rowsToRun;
    }

    protected boolean shouldLeaveAfterShot() {
        if (range == Range.CLOSE_RANGE
                && !shouldGoToPickupAfterShot()
                && !shouldGoToCloseLoopAfterShot()
                && !shouldGoToBackRowLoopAfterShot()) {
            return false;
        }
        return !shouldGoToPickupAfterShot()
                && !shouldGoToCloseLoopAfterShot()
                && !shouldGoToBackRowLoopAfterShot();
    }
}
