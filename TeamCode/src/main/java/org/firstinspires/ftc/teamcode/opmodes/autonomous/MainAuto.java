package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrent;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.follower;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.telemetryM;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
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
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.ShotPlan;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop;

class MainAuto extends OpMode {

    //Path Gen
    public Pose startPose;
    private final AutoPoses ap = new AutoPoses();
    private PathChain GoToPickup, Pickup, GoToScore, GoToLoad, LeavePath;

    //Enums
    private final Alliance alliance;
    private final Range range;
    private final Mode mode;
    private final ShotPlan shotPlan;
    private Range lastScoreRangeUsed;
    private enum PathRequest { GO_TO_PICKUP, PICKUP, GO_TO_SCORE, GO_TO_LOAD, LEAVE }

    public enum shootStates {
        INIT,
        PRESPIN,
        CLUTCHDOWN,
        WAIT1,
        SPIN,
        SPIN1,
        SPIN2,
        SPIN3,
        WAIT2
    }

    //State machine
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;
    private StateMachine shootAllMachine;

    //Robot and subsystems
    private Robot robot;

    //Other Variables
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentRowIndex = 0;
    private boolean stopRequested = false;
    private boolean preloadComplete = false;
    private static final double SHOOT_ACTION_SECONDS = 1.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 1.0;
    private static final double STATE_TIMEOUT_SECONDS = 2.0; // fallback: force state advance after this time
    private final ElapsedTime motifTimer = new ElapsedTime();
    private int acquiredMotifId = -1;
    private static final int MAX_ROWS = 4;
    private final ElapsedTime stateTimer = new ElapsedTime();

    boolean startShooting = false;

    MainAuto(Alliance alliance, Range range, Mode mode) {
        this(alliance, range, mode, ShotPlan.ALL_SELECTED);
    }

    MainAuto(Alliance alliance, Range range, Mode mode, ShotPlan shotPlan) {
        this.alliance = alliance;
        this.range = range;
        this.mode = mode;
        this.shotPlan = shotPlan;
        this.lastScoreRangeUsed = range;
        startPose = ap.findStartPose(alliance, range);
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        robot.drive.manualDrive = false;

        shootAllMachine = getShootAllMachine(robot);

        autoMachine = buildAutoMachine();

        robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (robot != null && robot.shooter != null) {
            int tagId = (alliance == Alliance.BLUE) ? 20 : 24;
            robot.shooter.setRequiredTagId(tagId);
        }

        rowsToRun = Math.min(resolveRowsForMode(mode), MAX_ROWS);
        rowsCompleted = 0;
        currentRowIndex = 0;
        preloadComplete = false;

        follower.setStartingPose(startPose);
        follower.update();

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
        stopRequested = false;

        autoMachine.start();
        robot.toInit();
//        shootAllMachine.start();
//        clutchSuperMachine.start();
    }

    @Override
    public void loop() {
        follower.update();

        robot.update();
        autoMachine.update();

        telemetryM.debug("Auto: " + this.getClass().getSimpleName() + " | State: " + activeState);
        telemetry.addData("Auto Action", getActionMessage());
        telemetry.addData("State Time", "%.2f", stateTimer.seconds());
        telemetryM.update(telemetry);
        telemetry.update();

        drawCurrentAndHistory();

        // Auto ends after we reach the final state and finish the park path.
//        if (!stopRequested && activeState == AutoStates.LEAVE && followerIdle()) {
//            stopRequested = true;
//            requestOpModeStop();
//        }
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
                GoToPickup = buildLinearPath(currentPose, ap.getPickupStart(alliance, range, currentRowIndex), true);
                break;
            case PICKUP:
                Pickup = buildLinearPath(currentPose, ap.getPickupEnd(alliance, range, currentRowIndex), false);
                break;
            case GO_TO_SCORE:
                lastScoreRangeUsed = getScoreRangeForCurrentShot();
                GoToScore = buildLinearPath(currentPose, getScorePoseForCurrentShot(), false);
                break;
            case GO_TO_LOAD:
                GoToLoad = buildLinearPath(currentPose, ap.getLoad(alliance), false);
                break;
            case LEAVE:
                LeavePath = buildLinearPath(currentPose, ap.getLeave(alliance, lastScoreRangeUsed), false);
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
                .state(AutoStates.ACQUIRE_MOTIF)
                .onEnter(this::onEnterAcquireMotif)
                .onExit(this::onExitAcquireMotif)
//                .transition(() -> motifAcquiredOrTimedOut() || stateTimedOut(), AutoStates.GO_TO_SHOOT)
                .transition(this::stateTimedOut, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
//                .transition(() -> shouldSkipShootPhase() || stateTimedOut(), AutoStates.LEAVE)
//                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_SHOOT)
                .transition(this::stateTimedOut, AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
//                .transition(() -> (shootActionComplete() || stateTimedOut()) && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
//                .transition(() -> (shootActionComplete() || stateTimedOut()) && !shouldStartNextCycle(), AutoStates.LEAVE)
                .transition(() -> stateTimedOut() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> stateTimedOut() && !shouldStartNextCycle(), AutoStates.LEAVE)

                .state(AutoStates.GO_TO_PICKUP)
                .onEnter(this::onEnterGoToPickup)
//                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_PICKUP)
                .transition(this::stateTimedOut, AutoStates.COMPLETE_PICKUP)

                .state(AutoStates.COMPLETE_PICKUP)
                .onEnter(this::onEnterCompletePickup)
                .onExit(this::onExitCompletePickup)
//                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.GO_TO_SHOOT) //Could add second condition of intake finished
                .transition(() -> followerIdle() && hasPendingRows(), AutoStates.GO_TO_SHOOT) //Could add second condition of intake finished
                .transition(() -> followerIdle() && !hasPendingRows(), AutoStates.LEAVE)

                .state(AutoStates.LEAVE)
                .onEnter(this::onEnterLeave)

                .build();
    }

    private void onEnterAcquireMotif() {
        setActiveState(AutoStates.ACQUIRE_MOTIF);

        motifTimer.reset();
        stateTimer.reset();

        coarseTurretAimAtObelisk();
    }

    private boolean motifAcquiredOrTimedOut() {
        boolean acquired = false;
        if (robot != null && robot.shooter != null && robot.shooter.vision != null) {
            acquired = robot.shooter.vision.hasTarget();
        }
        return acquired || motifTimer.seconds() >= MOTIF_ACQUIRE_TIMEOUT;
    }

    private void onExitAcquireMotif() {
        if (robot != null && robot.shooter != null && robot.shooter.vision != null) {
            acquiredMotifId = robot.shooter.vision.getCurrentTagId();
            robot.shooter.setMotifTagId(acquiredMotifId);
        }
    }

    private boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    private int getCurrentShotIndex() {
        int idx = preloadComplete ? rowsCompleted : 0;
        return Math.min(idx, 4); // clamp to known rows
    }

    private Range getScoreRangeForCurrentShot() {
        int shotIndex = getCurrentShotIndex();
        if (shotPlan == ShotPlan.CLOSEST_POINT) {
            boolean useClose;
            if (range == Range.LONG_RANGE) {
                useClose = shotIndex >= 3;
            } else {
                useClose = shotIndex <= 2;
            }
            return useClose ? Range.CLOSE_RANGE : Range.LONG_RANGE;
        }
        return range;
    }

    private Pose getScorePoseForCurrentShot() {
        return ap.getScore(alliance, getScoreRangeForCurrentShot());
    }

    private String getActionMessage() {
        switch (activeState) {
            case ACQUIRE_MOTIF:
                return "Looking for motif tag";
            case GO_TO_SHOOT:
                return "Driving to score (row " + (rowsCompleted + 1) + ")";
            case COMPLETE_SHOOT:
                return "Shooting row " + (rowsCompleted + 1);
            case GO_TO_PICKUP:
                return "Driving to pickup row " + (rowsCompleted + 1);
            case COMPLETE_PICKUP:
                return "Intaking row " + (rowsCompleted + 1);
            case LEAVE:
                return "Parking / leave path";
            default:
                return "Idle";
        }
    }

    private void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);

        stateTimer.reset();

        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }

        buildPath(PathRequest.GO_TO_SCORE);
        followPath(GoToScore);

        coarseTurretAimAtGoal();
        if (robot != null && robot.shooter != null) {
            robot.shooter.useTurretLock = true;
        }
    }

    private void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);

        stateTimer.reset();

        startShooting = true;
    }



    private void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0; // start counting rows after preload
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, MAX_ROWS);
        }
    }

    private void onEnterGoToPickup() {
        setActiveState(AutoStates.GO_TO_PICKUP);

        stateTimer.reset();

        refreshCurrentRowIndex();
        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(GoToPickup);
    }

    private void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);

        stateTimer.reset();

        buildPath(PathRequest.PICKUP);
        robot.intake.spinnerIn();
        followPath(Pickup, 0.2);
    }

    private void onExitCompletePickup() {
        robot.intake.spinnerZero();
    }

    private void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);

        stateTimer.reset();

        buildPath(PathRequest.LEAVE);
        followPath(LeavePath);
    }

    private void refreshCurrentRowIndex() {
        currentRowIndex = preloadComplete
                ? Math.max(0, Math.min(rowsCompleted, MAX_ROWS - 1))
                : 0;
    }

    private void followPath(PathChain path) {
        if (follower != null && path != null) {
            follower.followPath(path, true);
        }
    }

    private void followPath(PathChain path, double power) {
        if (follower != null && path != null) {
            follower.followPath(path, power, true);
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

    /** Determines if another row remains after the current shot completes. */
    private boolean shouldStartNextCycle() {
        if (!preloadComplete) {
            return rowsToRun > 0;
        }
        // rowsCompleted will be incremented when exiting COMPLETE_SHOOT; look ahead one.
        int nextCount = rowsCompleted + 1;
        return nextCount < rowsToRun;
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

    public PathChain buildLinearPath(Pose start, Pose end, boolean smoothEnd) {
        PathChain chain;
        if (smoothEnd) {
            chain = follower.pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                    .setBrakingStart(0.5)
                    .setBrakingStrength(0.8)
                    .build();
        }
        else
        {
            chain = follower.pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                    .build();
        }
        return chain;
    }

    public PathChain buildCurvedPath(Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public StateMachine getShootAllMachine (Robot robot){
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(shootStates.INIT)
                .transition(()->(startShooting), shootStates.PRESPIN)

                .state(shootStates.PRESPIN)
                .onEnter(()-> {
                    startShooting = false;
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                    if(transfer.desiredRotate == 1){
                        transfer.ballLeft();
                    } else if (transfer.desiredRotate == 2){
                        transfer.ballRight();
                    }
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN)
                .transitionTimed(2, shootStates.CLUTCHDOWN)

                .state(shootStates.CLUTCHDOWN)
                .onEnter(()-> {
                    transfer.max = 0.275;
                    transfer.setClutchBarelyDown();
                })
                .transitionTimed(0.1, shootStates.WAIT1)

                .state(shootStates.WAIT1)
                .transition(()-> shooter.isFarShot(), shootStates.SPIN1)
                .transition(() -> !shooter.isFarShot(), shootStates.SPIN)

                .state(shootStates.SPIN)
                .onEnter(()->{
                    transfer.max = 0.25;
                    transfer.ballLeft();
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.SPIN3)
                .transitionTimed(3, shootStates.SPIN3)

                .state(shootStates.SPIN1)
                .onEnter(()-> {
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.SPIN2)
                .transitionTimed(2, shootStates.SPIN2)

                .state(shootStates.SPIN2)
                .onEnter(()-> {
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.SPIN3)
                .transitionTimed(2, shootStates.SPIN3)

                .state(shootStates.SPIN3)
                .onEnter(()-> {
                    transfer.max = 0.275;
                    transfer.ballLeftSmall();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.WAIT2)
                .transitionTimed(2, shootStates.WAIT2)
                .onExit(()-> transfer.setClutchDownFar())

                .state(shootStates.WAIT2)
                .transitionTimed(0.7, shootStates.INIT)
                .onExit(()->{
                    transfer.setClutchUp();
                    transfer.ballRightSmall();
                    intake.spinnerMacroTarget = 0;
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.emptyBalls();
                    intake.spinnerMacro = false;
                    transfer.max = 0.4;
                })

                .build();
    }

    /** Returns the static goal pose in field (Pedro) coordinates. */
    private Pose getGoalPose() {
        if (alliance == Alliance.BLUE) {
            return new Pose(0, 144, Math.toRadians(90));
        } else {
            return new Pose(144, 144, Math.toRadians(90));
        }
    }

    /** Returns the obelisk pose in field coordinates (adjust if field measurements change). */
    private Pose getObeliskPose() {
        // Placeholder: aim near the alliance goal area; update to the true obelisk location if different.
        return new Pose(72, 144, Math.toRadians(90));
    }

    /** Wraps an angle in degrees to [-180, 180). */
    private double wrapDeg(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    /** Shared coarse aim helper to point turret from current pose toward a field target. */
    private void coarseTurretAimAt(Pose target) {
        if (robot == null || follower == null || robot.shooter == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null || target == null) return;
        robot.shooter.aimTurretAtFieldPose(
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getHeading(),
                target.getX(),
                target.getY());
    }

    /** Point turret toward the goal using chassis pose (coarse aim). */
    private void coarseTurretAimAtGoal() {
        coarseTurretAimAt(getGoalPose());
    }

    /** Point turret toward the obelisk using chassis pose (coarse aim). */
    private void coarseTurretAimAtObelisk() {
        coarseTurretAimAt(getObeliskPose());
    }
}
