package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrent;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.telemetryM;

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
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop;

class MainAuto extends OpMode {

    //Path Gen
    public Pose startPose;
    private final AutoPoses ap = new AutoPoses();
    private PathChain GoToPickup, Pickup, GoToScore, GoToLoad, LeavePath, ReleaseGoToPath, ReleaseCompletePath;
    double intakeSpeed = 0.25;

    //Enums
    private final Alliance alliance;
    private final Range range;
    private final Mode mode;
    private final ShotPlan shotPlan;
    private final boolean releaseAfterClosePickup;
    private Range lastScoreRangeUsed;
    private enum PathRequest { GO_TO_PICKUP, PICKUP, GO_TO_SCORE, GO_TO_LOAD, RELEASE_GO_TO, RELEASE_COMPLETE, LEAVE }

    public enum shootStates {
        INIT,
        PRESPIN,
        CLUTCHDOWN,
        WAIT1,
        SPIN,
        SPIN1,
        CLUTCHDOWN1,
        SPIN2,
        CLUTCHDOWN2,
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
    private final boolean skipEdgeRows = true;
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentRowIndex = 0;
    private boolean stopRequested = false;
    private boolean preloadComplete = false;
    private static final double SHOOT_ACTION_SECONDS = 10.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 2.0;
    private static final double STATE_TIMEOUT_SECONDS = 5.0; // fallback: force state advance after this time
    private final ElapsedTime motifTimer = new ElapsedTime();
    private int acquiredMotifId = -1;
    private static final int MAX_ROWS = 4;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shotDelayTimer = new ElapsedTime();

    boolean startShooting = false;
    boolean shootingComplete = false;
    private double shotDelaySeconds = 0.5;
    private double releaseIdleTimestamp = -1.0;

    MainAuto(Alliance alliance, Range range, Mode mode) {
        this(alliance, range, mode, ShotPlan.ALL_SELECTED);
    }

    MainAuto(Alliance alliance, Range range, Mode mode, ShotPlan shotPlan) {
        this(alliance, range, mode, shotPlan, true);
    }

    MainAuto(Alliance alliance, Range range, Mode mode, ShotPlan shotPlan, boolean releaseAfterClosePickup) {
        this.alliance = alliance;
        this.range = range;
        this.mode = mode;
        this.shotPlan = shotPlan;
        this.releaseAfterClosePickup = releaseAfterClosePickup;
        this.lastScoreRangeUsed = range;
        startPose = ap.findStartPose(alliance, range);
    }

    public double clutchDownTime = 0.1;
    public double clutchDownFarTime = 0.3;
    public double spinTime = 2.75;
    public double spinUpTimeout = 1.75;

    @Override
    public void init() {
        if (range == Range.CLOSE_RANGE) {
            robot = new Robot(hardwareMap, telemetry, false);
        }
        else {
            robot = new Robot(hardwareMap, telemetry, true);
        }

        robot.drive.manualDrive = false;

        shootAllMachine = getShootAllMachine(robot);

        autoMachine = buildAutoMachine();

        robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (range == Range.LONG_RANGE) {
            if (robot.getVoltage() > 12.6){
                intakeSpeed = 0.18;
            } else {
                intakeSpeed = 0.2;
            }
        }
        else {
            if (robot.getVoltage() > 12.6){
                intakeSpeed = 0.18;
            } else {
                intakeSpeed = 0.2;
            }
        }


        if (robot != null && robot.shooter != null) {
            int tagId = (alliance == Alliance.BLUE) ? 20 : 24;
            robot.shooter.setRequiredTagId(tagId);
            if(alliance == Alliance.BLUE) {
                GlobalVariables.allianceColor = "blue";
            } else {
                GlobalVariables.allianceColor = "red";
            }
        }

        rowsToRun = Math.min(resolveRowsForMode(mode), MAX_ROWS);
        if (skipEdgeRows) {
            if (range == Range.LONG_RANGE && rowsToRun > 0) {
                rowsToRun = Math.max(0, rowsToRun - 1); // skip row 1 on far side
            } else if (range == Range.CLOSE_RANGE && rowsToRun == MAX_ROWS) {
                rowsToRun = MAX_ROWS - 1; // skip row 4 on close side
            }
        }
        rowsCompleted = 0;
        currentRowIndex = 0;
        preloadComplete = false;

        follower.setStartingPose(startPose);
        follower.update();

        stateTimer.reset();

        robot.transfer.ballList[0] = "G";
        robot.transfer.ballList[1] = "P";
        robot.transfer.ballList[2] = "P";
        robot.transfer.numBalls = 3;
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
        shootAllMachine.start();
        if (range == Range.CLOSE_RANGE) {
            robot.shooter.hoodOffset -= 0; //0.045;
        }
    }

    @Override
    public void loop() {
        follower.update();
        updateTurretAim();

        robot.update();
        autoMachine.update();
        shootAllMachine.update();

        telemetryM.debug("Auto: " + this.getClass().getSimpleName() + " | State: " + activeState);
        telemetry.addData("Auto Action", getActionMessage());
        telemetry.addData("State Time", "%.2f", stateTimer.seconds());
        telemetry.addData("Current Row", currentRowIndex);
        telemetry.addData("Current Motif ID", acquiredMotifId);
        telemetry.addData("Target RPM", robot.shooter.targetRPM);
        telemetry.addData("Current RPM", robot.shooter.getShooterRPM());
        telemetry.addData("Sees desired tag?", robot.shooter.hasDesiredTarget);
        telemetry.addData("Turret Lock", robot.shooter.useTurretLock);
        telemetry.addData("Ball List", robot.transfer.balls);
        telemetry.addData("Shoot Order Number", robot.transfer.rotateOrder());
        telemetry.addData("Vision Error", robot.vision.getTx());
        telemetry.addData("ShootAll State", shootAllMachine.getState());
        telemetry.addData("Turret Pow", robot.shooter.turret.getPower());
        telemetry.addData("Is turret Tx In range?", Math.abs(robot.vision.getTx()) < 3);
        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Intake Speed", intakeSpeed);


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
            case RELEASE_GO_TO:
                ReleaseGoToPath = buildLinearPath(currentPose, ap.getReleaseGoTo(alliance, range), false);
                break;
            case RELEASE_COMPLETE:
                ReleaseCompletePath = buildLinearPath(currentPose, ap.getReleaseComplete(alliance, range), false);
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

    /** Maps the logical row count to the actual row index, respecting any skipped edge rows. */
    private int mapRowIndex(int logicalCount) {
        if (!skipEdgeRows) {
            return logicalCount;
        }
        if (range == Range.LONG_RANGE) {
            return logicalCount + 1; // skip row 1 (index 0) on far side
        }
        // close side: skip row 4 (index 3) by limiting rowsToRun; mapping stays the same
        return logicalCount;
    }

    private StateMachine buildAutoMachine() {
        return new StateMachineBuilder()
                .state(AutoStates.ACQUIRE_MOTIF)
                .onEnter(this::onEnterAcquireMotif)
                .onExit(this::onExitAcquireMotif)
                //.transition(this::motifAcquiredOrTimedOut, AutoStates.WAIT)
                .transition(this::motifAcquiredOrTimedOut, AutoStates.GO_TO_SHOOT)
                //.transitionTimed(4, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.WAIT)
                .transitionTimed(4, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(() -> shouldSkipShootPhase() || stateTimedOut(), AutoStates.LEAVE)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && !shouldStartNextCycle(), AutoStates.LEAVE)

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
            acquired = robot.shooter.vision.seesTag(21)
                    || robot.shooter.vision.seesTag(22)
                    || robot.shooter.vision.seesTag(23);
        }
        return acquired || motifTimer.seconds() >= MOTIF_ACQUIRE_TIMEOUT;
    }

    private void onExitAcquireMotif() {
        if (robot != null && robot.shooter != null && robot.shooter.vision != null) {
            // Determine motif ID. On short side, infer correct face from which tags are visible.
            if (range == Range.CLOSE_RANGE && !preloadComplete) {
                int seenId = robot.shooter.vision.getCurrentTagId();
                if (alliance == Alliance.BLUE) {
                    // Blue: subtract 1, except 21 -> 23
                    acquiredMotifId = (seenId == 21) ? 23 : (seenId - 1);
                } else {
                    // Red: add 1, except 23 -> 21
                    acquiredMotifId = (seenId == 23) ? 21 : (seenId + 1);
                }
            } else {
                acquiredMotifId = robot.shooter.vision.getCurrentTagId();
            }
            boolean validMotif = acquiredMotifId == 21 || acquiredMotifId == 22 || acquiredMotifId == 23;
            robot.shooter.setMotifTagId(validMotif ? acquiredMotifId : -1);
            if(validMotif){
                if(acquiredMotifId == 21){
                    GlobalVariables.motif = "GPP";
                } else if (acquiredMotifId == 22){
                    GlobalVariables.motif = "PGP";
                } else if (acquiredMotifId == 23){
                    GlobalVariables.motif = "PPG";
                }
            }
        }
    }

    private boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    private boolean shootTimedOut()
    {
        return stateTimer.seconds() >= SHOOT_ACTION_SECONDS;
    }

    private int getCurrentShotIndex() {
        int idx = preloadComplete ? mapRowIndex(rowsCompleted) : 0;
        return Math.min(idx, 4); // clamp to known rows
    }

    private Range getScoreRangeForShotIndex(int shotIndex) {
        if (shotPlan == ShotPlan.CLOSEST_POINT) {
            boolean useClose;
            if (range == Range.LONG_RANGE) {
                // For far start, switch to close aiming once we reach field row 3+
                useClose = shotIndex >= 2;
            } else {
                useClose = shotIndex <= 2;
            }
            return useClose ? Range.CLOSE_RANGE : Range.LONG_RANGE;
        }
        return range;
    }

    private Range getScoreRangeForCurrentShot() {
        int shotIndex = getCurrentShotIndex();
        return getScoreRangeForShotIndex(shotIndex);
    }

    /**
     * Range selection specifically for leave/park. For closest-point plans that switch to a close
     * shot late in a far-start run, choose leave based on the last field row shot.
     */
    private Range getLeaveRangeForShotIndex(int shotIndex) {
        if (shotPlan == ShotPlan.ALL_SELECTED) {
            return range;
        }
        if (range == Range.LONG_RANGE) {
            // Far start: rows 0-2 leave long, rows 3-4 leave close.
            return (shotIndex <= 2) ? Range.LONG_RANGE : Range.CLOSE_RANGE;
        }
        // Close start: rows 0-2 leave close, rows 3-4 leave long.
        return (shotIndex <= 2) ? Range.CLOSE_RANGE : Range.LONG_RANGE;
    }

    private Pose getScorePoseForCurrentShot() {
        Pose base = ap.getScore(alliance, getScoreRangeForCurrentShot());
        return base;
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
            case GO_TO_RELEASE:
                return "Driving to release lever";
            case COMPLETE_RELEASE:
                return "Releasing goal balls";
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

        robot.intake.spinnerMacro = true;
        robot.intake.spinnerMacroTarget = 0.95;
        robot.shooter.shooterShoot = true;
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(GoToScore);
    }

    private void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);

        stateTimer.reset();
        shotDelayTimer.reset();

        startShooting = true;
    }



    private void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0; // start counting rows after preload
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, MAX_ROWS);
        }
        shootingComplete = false;
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
        followPath(Pickup, intakeSpeed);
    }

    private void onExitCompletePickup() {
        robot.intake.spinnerZero();
    }

    private void onEnterGoToRelease() {
        setActiveState(AutoStates.GO_TO_RELEASE);

        stateTimer.reset();

        buildPath(PathRequest.RELEASE_GO_TO);
        followPath(ReleaseGoToPath);
    }

    private void onEnterCompleteRelease() {
        setActiveState(AutoStates.COMPLETE_RELEASE);

        stateTimer.reset();
        releaseIdleTimestamp = -1.0;

        buildPath(PathRequest.RELEASE_COMPLETE);
        followPath(ReleaseCompletePath, 0.35);
    }

    private void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);

        stateTimer.reset();

        // Park based on the range last used to score; fall back to the initially selected range.
        int lastShotIdx = preloadComplete ? mapRowIndex(Math.max(0, rowsCompleted - 1)) : 0;
        lastScoreRangeUsed = getLeaveRangeForShotIndex(lastShotIdx);
        buildPath(PathRequest.LEAVE);
        followPath(LeavePath);
    }

    private void refreshCurrentRowIndex() {
        int mapped = mapRowIndex(rowsCompleted);
        currentRowIndex = preloadComplete
                ? Math.max(0, Math.min(mapped, MAX_ROWS - 1))
                : mapRowIndex(0);
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

    /** Returns true if we should detour to the lever after collecting the first row on the close side. */
    private boolean shouldReleaseAfterPickup() {
        boolean closeSide = range == Range.CLOSE_RANGE;
        boolean modeAllows = mode != Mode.MOVE_ONLY && mode != Mode.PRELOAD_ONLY;
        boolean justFinishedFirstRow = preloadComplete && rowsCompleted == 0;
        return closeSide && modeAllows && justFinishedFirstRow && releaseAfterClosePickup;
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
        return shootingComplete;

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
                    .setBrakingStart(0.65)
                    .setBrakingStrength(0.85)
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

    //.transition(()->(startShooting && shooter.hasDesiredTarget), shootStates.PRESPIN)
    //startShooting = false;
    //shootingComplete = true;
    public StateMachine getShootAllMachine (Robot robot){
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(shootStates.INIT)
                .transition(() -> (startShooting && shooter.hasDesiredTarget && Math.abs(robot.vision.getTx()) < 6), shootStates.PRESPIN)

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
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN)

                .state(shootStates.CLUTCHDOWN)
                .onEnter(()-> {
                    transfer.max = 0.275;
                    transfer.setClutchBarelyDown();
                })
                .transitionTimed(clutchDownTime, shootStates.WAIT1)

                .state(shootStates.WAIT1)
                .transition(()-> shooter.isFarShot(), shootStates.SPIN1)
                .transition(() -> !shooter.isFarShot(), shootStates.SPIN)

                .state(shootStates.SPIN)
                .onEnter(()->{
                    transfer.max = 0.2;
                    transfer.ballLeft();
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.SPIN3)
                .transitionTimed(spinTime, shootStates.SPIN3)

                .state(shootStates.SPIN1)
                .onEnter(()-> {
                    transfer.ballLeftSmall();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN1)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN1)

                .state(shootStates.CLUTCHDOWN1)
                .onEnter(()->transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN2)
                .onExit(()->transfer.setClutchBarelyDown())

                .state(shootStates.SPIN2)
                .onEnter(()-> {
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN2)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN2)

                .state(shootStates.CLUTCHDOWN2)
                .onEnter(()->transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN3)
                .onExit(()-> {
                    transfer.setClutchBarelyDown();
                    transfer.ballRightSmall();
                })

                .state(shootStates.SPIN3)
                .onEnter(()-> {
                    transfer.max = 0.275;
                    transfer.ballLeftSmall();
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.WAIT2)
                .transitionTimed(spinUpTimeout, shootStates.WAIT2)
                .onExit(()-> transfer.setClutchDownFar())

                .state(shootStates.WAIT2)
                .transitionTimed(clutchDownFarTime, shootStates.INIT)
                .onExit(()->{
                    transfer.setClutchUp();
                    transfer.ballRightSmall();
                    intake.spinnerMacroTarget = 0;
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.emptyBalls();
                    intake.spinnerMacro = false;
                    transfer.max = 0.4;
                    shootingComplete = true;
                })

                .build();
    }

    /** Returns the static goal pose in field (Pedro) coordinates. */
    private Pose getGoalPose() {
        if (alliance == Alliance.BLUE) {
            if (!preloadComplete && (range == Range.LONG_RANGE)) {
                return new Pose(0-1.25, 144, Math.toRadians(90));
            }
            else if (preloadComplete && (range == Range.LONG_RANGE)) {
                return new Pose(0+5, 144, Math.toRadians(90));
            }
            else if (!preloadComplete && (range == Range.CLOSE_RANGE)) {
                return new Pose(0, 144, Math.toRadians(90));
            }
            else if (preloadComplete && (range == Range.CLOSE_RANGE)){
                return new Pose(0-5, 144, Math.toRadians(90));
            }
            else {
                return new Pose (0, 0, 0);
            }
        } else {
            if (!preloadComplete && (range == Range.LONG_RANGE)) {
                return new Pose(144+1.25, 144, Math.toRadians(90));
            }
            else if (preloadComplete && (range == Range.LONG_RANGE)) {
                return new Pose(144-4, 144, Math.toRadians(90));
            }
            else if (!preloadComplete && (range == Range.CLOSE_RANGE)) {
                return new Pose(144, 144, Math.toRadians(90));
            }
            else if (preloadComplete && (range == Range.CLOSE_RANGE)){
                return new Pose(144, 144, Math.toRadians(90));
            }
            else {
                return new Pose (0, 0, 0);
            }
        }
    }

    /** Returns the obelisk pose in field coordinates (adjust if field measurements change). */
    private Pose getObeliskPose() {
        // Placeholder: aim near the alliance goal area; update to the true obelisk location if different.
        return new Pose(72, 144, Math.toRadians(90));
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

    /** Waits 1s after the release path finishes before advancing. */
    private boolean releaseWaitDone() {
        if (stateTimedOut()) return true;
        boolean idle = followerIdle();
        double now = stateTimer.seconds();
        if (idle) {
            if (releaseIdleTimestamp < 0.0) {
                releaseIdleTimestamp = now;
            }
            return (now - releaseIdleTimestamp) >= 1.0;
        }
        // not idle yet; reset marker
        releaseIdleTimestamp = -1.0;
        return false;
    }

    /** Keep turret target refreshed while driving */
    private void updateTurretAim() {
        if (activeState == AutoStates.ACQUIRE_MOTIF) {
            robot.shooter.useTurretLock = false;
            coarseTurretAimAtObelisk();
            telemetry.addData("Obelisk Aim", true);
        }
//        else if (activeState == AutoStates.GO_TO_SHOOT || activeState == AutoStates.COMPLETE_SHOOT) { //IF BREAKS STUFF, SWITCH BACK TO JUST "ELSE"
        else if (robot.shooter.hasDesiredTarget) {
            robot.shooter.useTurretLock = true;
            telemetry.addData("Lock Aim", true);
        }
        else {
            robot.shooter.useTurretLock = false;
            robot.shooter.turretLockController.reset();
            coarseTurretAimAtGoal();
            telemetry.addData("Goal Aim", true);
        }
    }
}
