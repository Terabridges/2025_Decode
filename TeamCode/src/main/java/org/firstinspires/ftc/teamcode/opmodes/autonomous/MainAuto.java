package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrent;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;
import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.telemetryM;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoIntakeSpeed;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoMotifTracker;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPathLibrary;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoShootMachine;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoTurretAim;
import org.firstinspires.ftc.teamcode.config.autoUtil.ReleaseWaiter;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.ShotPlan;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

@PsiKitAutoLog(rlogPort = 5802)
class MainAuto extends OpMode {

    // ===== Pathing Configuration =====
    private final AutoPoses poses = new AutoPoses();
    private final AutoPathLibrary pathLibrary = new AutoPathLibrary(poses);
    private final Pose startPose;
    private PathChain goToPickupPath;
    private PathChain pickupPath;
    private PathChain goToScorePath;
    private PathChain leavePath;
    private PathChain releaseGoToPath;
    private PathChain releaseCompletePath;
    private double intakeSpeed = 0.25;
    private final AutoIntakeSpeed intakeSpeedModel = new AutoIntakeSpeed(
            -0.02, 0.47, 0.18, 0.24, -0.01, 0.01);

    // ===== Constants =====
    private static final int MAX_ROWS = 3;
    private static final double SHOOT_ACTION_SECONDS = 10.0;
    private static final double MOTIF_ACQUIRE_TIMEOUT = 1.0;
    private static final double STATE_TIMEOUT_SECONDS = 5.0; // fallback: force state advance after this time
    private static final int TAG_BLUE = 20;
    private static final int TAG_RED = 24;
    private static final double RELEASE_IDLE_SECONDS = 1.0;
    private static final double RELEASE_TIMEOUT_SECONDS = 2.25;

    private final Alliance alliance;
    private final Range range;
    private final Mode mode;
    private final ShotPlan shotPlan;
    private final boolean releaseAfterClosePickup;
    private Range lastScoreRangeUsed;
    private enum PathRequest { GO_TO_PICKUP, PICKUP, GO_TO_SCORE, RELEASE_GO_TO, RELEASE_COMPLETE, LEAVE }

    // ===== State Machine =====
    private StateMachine autoMachine;
    private AutoStates activeState = AutoStates.ACQUIRE_MOTIF;
    private AutoShootMachine shootMachine;
    private AutoTurretAim turretAim;

    // ===== Robot and Subsystems =====
    private Robot robot;

    // ===== Runtime State =====
    private int rowsToRun = 0;
    private int rowsCompleted = 0;
    private int currentRowIndex = 0;
    private boolean preloadComplete = false;
    private AutoMotifTracker motifTracker;
    private int acquiredMotifId = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private ReleaseWaiter releaseWaiter = new ReleaseWaiter(RELEASE_IDLE_SECONDS);

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
        startPose = poses.findStartPose(alliance, range);
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

        robot.drive.manualDrive = false;

        shootMachine = new AutoShootMachine(robot, clutchDownTime, clutchDownFarTime, spinTime, spinUpTimeout);
        turretAim = new AutoTurretAim(robot, poses, alliance, range, telemetry);
        motifTracker = new AutoMotifTracker(robot, alliance, range, MOTIF_ACQUIRE_TIMEOUT);

        autoMachine = buildAutoMachine();

        robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeSpeed = intakeSpeedModel.compute(robot.getVoltage(), alliance, range);

        if (robot != null && robot.shooter != null) {
            int tagId = (alliance == Alliance.BLUE) ? TAG_BLUE : TAG_RED;
            robot.shooter.setRequiredTagId(tagId);
            if(alliance == Alliance.BLUE) {
                GlobalVariables.allianceColor = "blue";
            } else {
                GlobalVariables.allianceColor = "red";
            }
        }

        rowsToRun = Math.min(resolveRowsForMode(mode), MAX_ROWS);
        rowsCompleted = 0;
        currentRowIndex = 0;
        preloadComplete = false;

        FollowerManager.initFollower(hardwareMap, startPose);
        GlobalVariables.autoFollowerValid = false;

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
//        telemetry.addData("Current Row", currentRowIndex);
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
        GlobalVariables.autoFollowerValid = (follower != null);
    }

    // ===== State Machine Construction =====
    private int resolveRowsForMode(Mode mode) {
        switch (mode) {
            case ONE_ROW:
                return 1;
            case TWO_ROW:
                return 2;
            case THREE_ROW:
                return 3;
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
                .transition(this::motifAcquiredOrTimedOut, AutoStates.GO_TO_SHOOT)

                .state(AutoStates.GO_TO_SHOOT)
                .onEnter(this::onEnterGoToShoot)
                .transition(() -> shouldSkipShootPhase() || stateTimedOut(), AutoStates.LEAVE)
                .transition(() -> followerIdle() || stateTimedOut(), AutoStates.COMPLETE_SHOOT)

                .state(AutoStates.COMPLETE_SHOOT)
                .onEnter(this::onEnterCompleteShoot)
                .onExit(this::onExitCompleteShoot)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && shouldStartNextCycle(), AutoStates.GO_TO_PICKUP)
                .transition(() -> (shootActionComplete() || shootTimedOut()) && followerIdle() && !shouldStartNextCycle(), AutoStates.LEAVE)

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

    // ===== State Machine Callbacks =====
    private void onEnterAcquireMotif() {
        setActiveState(AutoStates.ACQUIRE_MOTIF);

        motifTracker.reset();
        resetStateTimer();

        turretAim.aimAtObelisk();
    }

    private boolean motifAcquiredOrTimedOut() {
        return motifTracker.hasMotifOrTimedOut();
    }

    private void onExitAcquireMotif() {
        motifTracker.resolveMotif(preloadComplete);
        acquiredMotifId = motifTracker.getAcquiredMotifId();
    }

    private void onEnterGoToShoot() {
        setActiveState(AutoStates.GO_TO_SHOOT);

        resetStateTimer();

        if (!preloadComplete && !shouldShootPreload()) {
            return;
        }

        // Start spin-up early so we arrive ready to shoot.
        robot.intake.spinnerMacro = true;
        robot.intake.spinnerMacroTarget = 0.95;
        robot.shooter.shooterShoot = true;
        robot.shooter.useTurretLock = true;
        buildPath(PathRequest.GO_TO_SCORE);
        followPath(goToScorePath);
    }

    private void onEnterCompleteShoot() {
        setActiveState(AutoStates.COMPLETE_SHOOT);

        resetStateTimer();
        shootTimer.reset();
        shootMachine.requestShoot();
    }

    private void onExitCompleteShoot() {
        if (!preloadComplete) {
            preloadComplete = true;
            rowsCompleted = 0; // start counting rows after preload
        } else {
            rowsCompleted = Math.min(rowsCompleted + 1, MAX_ROWS);
        }
        shootMachine.clearShootingComplete();
    }

    private void onEnterGoToPickup() {
        setActiveState(AutoStates.GO_TO_PICKUP);

        resetStateTimer();

        refreshCurrentRowIndex();
        buildPath(PathRequest.GO_TO_PICKUP);
        followPath(goToPickupPath);
    }

    private void onEnterCompletePickup() {
        setActiveState(AutoStates.COMPLETE_PICKUP);

        resetStateTimer();

        buildPath(PathRequest.PICKUP);
        robot.intake.spinnerIn();
        followPath(pickupPath, intakeSpeed);
    }

    private void onExitCompletePickup() {
        robot.intake.spinnerZero();
    }

    private void onEnterGoToRelease() {
        setActiveState(AutoStates.GO_TO_RELEASE);

        resetStateTimer();

        buildPath(PathRequest.RELEASE_GO_TO);
        followPath(releaseGoToPath);
    }

    private void onEnterCompleteRelease() {
        setActiveState(AutoStates.COMPLETE_RELEASE);

        resetStateTimer();
        releaseWaiter.reset();

        buildPath(PathRequest.RELEASE_COMPLETE);
        followPath(releaseCompletePath, 0.35);
    }

    private void onEnterLeave() {
        setActiveState(AutoStates.LEAVE);

        resetStateTimer();

        // Park based on the range last used to score; fall back to the initially selected range.
        int lastShotIdx = preloadComplete ? Math.max(0, rowsCompleted) : 0;
        lastScoreRangeUsed = getLeaveRangeForShotIndex(lastShotIdx);
        buildPath(PathRequest.LEAVE);
        followPath(leavePath);
    }

    // ===== Path Building =====
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
                goToPickupPath = pathLibrary.goToPickup(currentPose, alliance, range, currentRowIndex);
                break;
            case PICKUP:
                pickupPath = pathLibrary.pickup(currentPose, alliance, range, currentRowIndex);
                break;
            case GO_TO_SCORE:
                lastScoreRangeUsed = getScoreRangeForCurrentShot();
                goToScorePath = pathLibrary.goToScore(currentPose, getScorePoseForCurrentShot());
                break;
            case RELEASE_GO_TO:
                releaseGoToPath = pathLibrary.releaseGoTo(currentPose, alliance, range);
                break;
            case RELEASE_COMPLETE:
                releaseCompletePath = pathLibrary.releaseComplete(currentPose, alliance, range);
                break;
            case LEAVE:
                leavePath = pathLibrary.leave(currentPose, alliance, lastScoreRangeUsed);
                break;
        }
    }

    // ===== Row and Shot Planning =====
    private void refreshCurrentRowIndex() {
        currentRowIndex = preloadComplete
                ? Math.max(0, Math.min(rowsCompleted, MAX_ROWS - 1))
                : 0;
    }

    private int getCurrentShotIndex() {
        // shotIndex: 0 = preload, 1 = row1, 2 = row2, 3 = row3
        int idx = preloadComplete ? (rowsCompleted + 1) : 0;
        return Math.min(idx, MAX_ROWS);
    }

    private Range getScoreRangeForShotIndex(int shotIndex) {
        if (shotPlan == ShotPlan.CLOSEST_POINT) {
            boolean useClose;
            if (range == Range.LONG_RANGE) {
                // Far start: preload + row1 far, row2+ close.
                useClose = shotIndex >= 2;
            } else {
                // Close start: preload + rows1-2 close, row3+ far.
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
            // Far start: preload + row1 leave long, row2+ leave close.
            return (shotIndex <= 1) ? Range.LONG_RANGE : Range.CLOSE_RANGE;
        }
        // Close start: preload + rows1-2 leave close, row3+ leave long.
        return (shotIndex <= 2) ? Range.CLOSE_RANGE : Range.LONG_RANGE;
    }

    private Pose getScorePoseForCurrentShot() {
        Pose base = poses.getScore(alliance, getScoreRangeForCurrentShot());
        return base;
    }

    // ===== Follower Helpers =====
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

    // ===== Decision Helpers =====
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

    /** Determines if another row remains after the current shot completes. */
    private boolean shouldStartNextCycle() {
        if (!preloadComplete) {
            return rowsToRun > 0;
        }
        // rowsCompleted will be incremented when exiting COMPLETE_SHOOT; look ahead one.
        int nextCount = rowsCompleted + 1;
        return nextCount < rowsToRun;
    }

    private boolean shouldSkipShootPhase() {
        return !preloadComplete && !shouldShootPreload();
    }

    // ===== State/Timer Helpers =====
    private boolean stateTimedOut() {
        return stateTimer.seconds() >= STATE_TIMEOUT_SECONDS;
    }

    private boolean shootTimedOut() {
        return shootTimer.seconds() >= SHOOT_ACTION_SECONDS;
    }

    private boolean shootActionComplete() {
        if (!preloadComplete && !shouldShootPreload()) {
            return true;
        }
        return shootMachine.isShootingComplete();

        //TODO get a boolean from shooter subsystem
    }

    private void setActiveState(AutoStates state) {
        activeState = state;
    }

    private void resetStateTimer() {
        stateTimer.reset();
    }

    /** Waits 1s after the release path finishes before advancing. */
    private boolean releaseWaitDone() {
        return releaseWaiter.isDone(followerIdle(), RELEASE_TIMEOUT_SECONDS);
    }

    // ===== Telemetry Helpers =====
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

}
