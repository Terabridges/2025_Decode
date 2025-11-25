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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;

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

    private enum ShootState { INIT, CLUTCHDOWN, SPIN, CLUTCHDOWNFAR }
    private enum ClutchState { INIT, CLUTCHDOWN, CLUTCHUP }

    //State machine
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.GO_TO_SHOOT;
    private StateMachine shootAllMachine;
    private StateMachine clutchSuperMachine;

    //Robot and subsystems
    private Robot robot;

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
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        shootAllMachine = getShootAllMachine(robot);
        clutchSuperMachine = getClutchSuperMachine(robot);

        autoMachine = buildAutoMachine();

        robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rowsToRun = Math.min(resolveRowsForMode(mode), MAX_ROWS);
        rowsCompleted = 0;
        currentRowIndex = 0;
        preloadComplete = false;

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
        follower.setStartingPose(startPose);
        follower.update();
        stopRequested = false;
        if (autoMachine != null) {
            autoMachine.start();
        }

        robot.toInit();
        shootAllMachine.start();
        clutchSuperMachine.start();
    }

    @Override
    public void loop() {
        follower.update();

        robot.update();
        autoMachine.update();
        stateMachinesUpdate();

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
        //TODO Call shoot method here
    }

    private void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, MAX_ROWS);
        }
        //TODO Call end shoot method here
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
        robot.intake.spinnerIn();
        followPath(Pickup);
    }

    private void onExitCompletePickup() {
        robot.intake.spinnerZero();
    }

    private void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);
        //TODO build and follow leave path (ensure we are outside of triangle)
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

    public void stateMachinesUpdate(){
        shootAllMachine.update();
        clutchSuperMachine.update();
    }

    //TODO Implement auto version without gamepad
    public StateMachine getShootAllMachine (Robot robot){
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(ShootState.INIT)
                .transition(()->(true), ShootState.CLUTCHDOWN)

                .state(ShootState.CLUTCHDOWN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    transfer.setClutchDown();
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                })
                .transitionTimed(0.5, ShootState.SPIN)

                .state(ShootState.SPIN)
                .onEnter(()-> {
                    transfer.ballRight();
                    transfer.ballRight();
                    transfer.ballRightSmall();
                })
                .transitionTimed(3.2, ShootState.CLUTCHDOWNFAR)

                .state(ShootState.CLUTCHDOWNFAR)
                .onEnter(()-> {
                    transfer.setClutchDownFar();
                })
                .transitionTimed(1.5, ShootState.INIT)
                .onExit(()-> {
                    transfer.setClutchUp();
                    intake.spinnerMacroTarget = 0;
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.ballLeftSmall();
                    transfer.emptyBalls();
                    intake.spinnerMacro = false;
                })

                .build();
    }

    public StateMachine getClutchSuperMachine (Robot robot){
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(ClutchState.INIT)
                .transition(()->(true), ClutchState.CLUTCHDOWN)

                .state(ClutchState.CLUTCHDOWN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    transfer.setClutchDownFar();
                })
                .transitionTimed(1.2, ClutchState.CLUTCHUP)

                .state(ClutchState.CLUTCHUP)
                .onEnter(()->{
                    intake.spinnerMacro = false;
                    intake.spinnerMacroTarget = 0;
                    transfer.setClutchUp();
                })
                .transitionTimed(0.2, ClutchState.INIT)

                .build();
    }
}
