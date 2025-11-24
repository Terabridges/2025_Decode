package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrent;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.follower;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.telemetryM;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

class MainAuto extends OpMode {

    //Path Gen
    public Pose startPose;
    private final AutoPoses ap = new AutoPoses();
    private PathChain GoToPickup, Pickup, GoToScore, GoToLoad;

    //Enums
    private final Alliance alliance;
    private final Range range;
    private final Mode mode;
    private enum PathRequest { GO_TO_PICKUP, PICKUP, GO_TO_SCORE, GO_TO_LOAD }

    //State machine
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.GO_TO_SHOOT;

    //Other Variables
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentRowIndex = 0;
    private boolean stopRequested = false;
    private boolean preloadComplete = false;
    private static final double SHOOT_ACTION_SECONDS = 1.0;
    private static final int MAX_ROWS = 4;
    private final ElapsedTime stateTimer = new ElapsedTime();

    MainAuto(Alliance alliance, Range range, Mode mode) {
        this.alliance = alliance;
        this.range = range;
        this.mode = mode;
        startPose = ap.findStartPose(alliance, range);
    }

    @Override
    public void init() {
        rowsToRun = Math.min(resolveRowsForMode(mode), MAX_ROWS);
        rowsCompleted = 0;
        currentRowIndex = 0;
        preloadComplete = false;
        stateTimer.reset();
        autoMachine = buildAutoMachine();
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
        follower.setStartingPose(startPose);
        follower.update();
        stopRequested = false;
        if (autoMachine != null) {
            autoMachine.start();
        }
    }

    @Override
    public void loop() {
        follower.update();
        if (autoMachine != null) {
            autoMachine.update();
        }
        drawCurrentAndHistory();

        // Auto ends after we reach the final state and finish the park path.
        if (!stopRequested && activeState == AutoStates.LEAVE && followerIdle()) {
            stopRequested = true;
            requestOpModeStop();
        }
    }

    private void buildPath(PathRequest request) {
        if (follower == null) {
            return;
        }

        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return;
        }

        switch (request) {
            case GO_TO_PICKUP:
                GoToPickup = buildLinearPath(currentPose, ap.getPickupStart(alliance, range, currentRowIndex));
                break;
            case PICKUP:
                Pickup = buildLinearPath(currentPose, ap.getPickupEnd(alliance, range, currentRowIndex));
                break;
            case GO_TO_SCORE:
                GoToScore = buildLinearPath(currentPose, ap.getScore(alliance, range));
                break;
            case GO_TO_LOAD:
                GoToLoad = buildLinearPath(currentPose, ap.getLoad(alliance));
                break;
        }
    }

    private int resolveRowsForMode(Mode mode) {
        switch (mode) {
            case ONE_ROW:
                return 1;
            case TWO_ROW:
                return 2;
            case THREE_ROW:
                return 3;
            case FOUR_ROW:
                return 4;
            case MOVE_ONLY:
            case PRELOAD_ONLY:
            default:
                return 0;
        }
    }

    private StateMachine buildAutoMachine() {
        return new StateMachineBuilder()
                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(this::shouldSkipShootPhase, AutoStates.LEAVE)
                .transition(this::followerIdle, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> shootActionComplete() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> shootActionComplete() && !shouldStartNextCycle(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
                .transition(this::followerIdle, AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .onExit(this::onExitCompletePickup)
                .transition(this::followerIdle, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    private void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);
        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(GoToScore);
    }

    private void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);
        stateTimer.reset();
        //Call shoot method here
    }

    private void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, MAX_ROWS);
        }
        //Call end shoot method here
    }

    private void onEnterGoToPickup() {
        setActiveState(AutoStates.GO_TO_PICKUP);
        refreshCurrentRowIndex();
        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(GoToPickup);
    }

    private void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);
        buildPath(PathRequest.PICKUP);
        //Activate intake
        followPath(Pickup);
    }

    private void onExitCompletePickup() {
        //Deactivate intake
    }

    private void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);
        // build and follow leave path (ensure we are outside of triangle)
    }

    private void refreshCurrentRowIndex() {
        currentRowIndex = Math.min(rowsCompleted, MAX_ROWS - 1);
    }

    private void followPath(PathChain path) {
        if (follower != null && path != null) {
            follower.followPath(path);
        }
    }

    private boolean followerIdle() {
        return follower != null && !follower.isBusy();
    }

    private boolean shouldShootPreload() {
        return mode != Mode.MOVE_ONLY;
    }

    private boolean hasPendingRows() {
        return rowsCompleted < rowsToRun;
    }

    private boolean shouldStartNextCycle() {
        if (!preloadComplete) {
            return rowsToRun > 0;
        }
        return hasPendingRows();
    }

    private boolean shootActionComplete() {
        if (!preloadComplete && !shouldShootPreload()) {
            return true;
        }
        return stateTimer.seconds() >= SHOOT_ACTION_SECONDS;

        //TODO get a boolean from shooter subsystem
    }

    private void setActiveState(AutoStates state) {
        activeState = state;
    }

    private boolean shouldSkipShootPhase() {
        return !preloadComplete && !shouldShootPreload();
    }

    public PathChain buildLinearPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public PathChain buildCurvedPath(Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
