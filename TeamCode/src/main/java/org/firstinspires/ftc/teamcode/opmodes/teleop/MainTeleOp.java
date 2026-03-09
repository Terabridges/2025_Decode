package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.drawCurrentAndHistory;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.control.Intake.ClutchControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpindexControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpinnerControl;
import org.firstinspires.ftc.teamcode.config.control.Other.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.Other.LiftControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.LightsControl;
import org.firstinspires.ftc.teamcode.config.control.Other.OtherControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.OuttakeControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.TurretControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.VisionControl;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.LoopTimeTracker;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN * 0.5;
    public static boolean enableSectionTimingLogs = true;
    public static double autoOffsetStationarySeconds = 1.0;
    // Match the auto shooting "robot settled" gate.
    public static double autoOffsetMaxRobotSpeedInS = 1.5;
    public static double autoOffsetMaxRobotAngularSpeedDegS = 12.0;

    IntakeControl intakeControl;
    OuttakeControl outtakeControl;
    OtherControl otherControl;
    ClutchControl clutchControl;
    SpindexControl spindexControl;
    SpinnerControl spinnderControl;
    LiftControl liftControl;
    LightsControl lightsControl;
    DriveControl driveControl;
    ShooterControl shooterControl;
    TurretControl turretControl;
    VisionControl visionControl;

    List<Control> controls;
    Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    StateMachine shootAllMachine;
    StateMachine sortingShootAllMachine;
    private boolean shootRequestPending = false;
    private boolean pendingShootUsesSorting = false;

    private JoinedTelemetry joinedTelemetry;
    private LoopTimeTracker loopTimeTracker;
    private final ElapsedTime autoOffsetStationaryTimer = new ElapsedTime();
    private boolean autoOffsetWaitingForMovement = false;

    public ElapsedTime telemetryTimer;
    public double telemetryTime;

    EdgeDetector getReadyShoot = new EdgeDetector(() -> robot.getReadyShoot());
    EdgeDetector toggleSorting = new EdgeDetector(()-> robot.toggleSorting());
    EdgeDetector nextMotif = new EdgeDetector(()-> GlobalVariables.nextMotif());
    EdgeDetector flashLights = new EdgeDetector(()-> robot.toggleLightsTurret());

    @Override
    public void init() {
        configureLowOverheadPsiKitLogging();

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
        outtakeControl = new OuttakeControl(robot, gamepad1, gamepad2);
        otherControl = new OtherControl(robot, gamepad1, gamepad2);
        clutchControl = new ClutchControl(robot, gamepad1, gamepad2);
        spindexControl = new SpindexControl(robot, gamepad1, gamepad2);
        spinnderControl = new SpinnerControl(robot, gamepad1, gamepad2);
        liftControl = new LiftControl(robot, gamepad1, gamepad2);
        lightsControl = new LightsControl(robot, gamepad1, gamepad2);
        driveControl = new DriveControl(robot, gamepad1, gamepad2);
        shooterControl = new ShooterControl(robot, gamepad1, gamepad2);
        turretControl = new TurretControl(robot, gamepad1, gamepad2);
        visionControl = new VisionControl(robot, gamepad1, gamepad2);


        controls = new ArrayList<>(Arrays.asList(intakeControl, outtakeControl, otherControl, clutchControl, spindexControl, spinnderControl, liftControl, lightsControl, driveControl, shooterControl, turretControl, visionControl));

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        shootAllMachine = robot.getShootAllMachine();
        sortingShootAllMachine = robot.getSortedShootAllMachine();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        loopTimeTracker = new LoopTimeTracker();
        telemetryTimer = new ElapsedTime();

    }

    private void configureLowOverheadPsiKitLogging() {
        FtcLogTuning.nonBulkReadPeriodSec = 0.10;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;
        FtcLogTuning.pinpointLoggerCallsUpdate = false;
        FtcLogTuning.pinpointReadPeriodSec = .10;
    }

    @Override
    public void init_loop(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.a && !previousGamepad1.a){
            GlobalVariables.nextMotif();
        }

        if (currentGamepad1.b && !previousGamepad1.b){
            GlobalVariables.toggleAlliance();
        }

        telemetry.addData("Press A to change Motif. Press B to change alliance color.", "");
        telemetry.addData("Motif", GlobalVariables.getMotif());
        telemetry.addData("Alliance Color", GlobalVariables.getAllianceColor());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.toInit();
        applyAllianceVisionLockConfig();
        boolean reuseAutoFollower = GlobalVariables.isAutoFollowerValid()
                && FollowerManager.follower != null;
        if (reuseAutoFollower) {
            FollowerManager.getFollower(hardwareMap);
        } else {
            Alliance alliance = GlobalVariables.isBlueAlliance() ? Alliance.BLUE : Alliance.RED;
            Pose teleopStartPose = new AutoPoses().findStartPose(alliance, Range.LONG_RANGE);
            FollowerManager.initFollower(hardwareMap, teleopStartPose);
        }
        // Consume the auto->teleop handoff flag for this start.
        GlobalVariables.setAutoFollowerValid(false);
        robot.outtake.setAimLockEnabled(true);

        shootAllMachine.start();
        sortingShootAllMachine.start();
        autoOffsetStationaryTimer.reset();
        autoOffsetWaitingForMovement = false;

        loopTimeTracker.reset();
        telemetryTimer.reset();
    }

    @Override
    public void loop() {
        long tLoopStartNs = System.nanoTime();

        gamepadUpdate();
        long tAfterGamepadNs = System.nanoTime();

        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }
        long tAfterFollowerNs = System.nanoTime();

        updateAllianceToggle();
        applyAllianceVisionLockConfig();
        long tAfterAllianceNs = System.nanoTime();

        controlsUpdate();
        long tAfterControlsNs = System.nanoTime();

        robot.update();
        autoUpdateTurretAimOffsetWhenSettled();
        long tAfterRobotNs = System.nanoTime();

        logPsiKitData();
        controlsTelemetryUpdate();
        long tAfterTelemetryNs = System.nanoTime();

        stateMachinesUpdate();
        long tAfterStateMachinesNs = System.nanoTime();

        //drawCurrentAndHistory();
        long tAfterDrawingNs = System.nanoTime();

        loopTimeTracker.sampleLoop();
        long tLoopEndNs = System.nanoTime();

        if (enableSectionTimingLogs) {
            Logger.recordOutput("MainTeleOp/TimingMs/GamepadUpdate", nanosToMillis(tAfterGamepadNs - tLoopStartNs));
            Logger.recordOutput("MainTeleOp/TimingMs/FollowerUpdate", nanosToMillis(tAfterFollowerNs - tAfterGamepadNs));
            Logger.recordOutput("MainTeleOp/TimingMs/AllianceConfig", nanosToMillis(tAfterAllianceNs - tAfterFollowerNs));
            Logger.recordOutput("MainTeleOp/TimingMs/ControlsUpdate", nanosToMillis(tAfterControlsNs - tAfterAllianceNs));
            Logger.recordOutput("MainTeleOp/TimingMs/RobotUpdate", nanosToMillis(tAfterRobotNs - tAfterControlsNs));
            Logger.recordOutput("MainTeleOp/TimingMs/ControlsTelemetry", nanosToMillis(tAfterTelemetryNs - tAfterRobotNs));
            Logger.recordOutput("MainTeleOp/TimingMs/StateMachines", nanosToMillis(tAfterStateMachinesNs - tAfterTelemetryNs));
            Logger.recordOutput("MainTeleOp/TimingMs/DrawField", nanosToMillis(tAfterDrawingNs - tAfterStateMachinesNs));
            Logger.recordOutput("MainTeleOp/TimingMs/LoopTrackerSample", nanosToMillis(tLoopEndNs - tAfterDrawingNs));
            Logger.recordOutput("MainTeleOp/TimingMs/TotalLoop", nanosToMillis(tLoopEndNs - tLoopStartNs));
            Logger.recordOutput("COMMANDEDPOS", robot.intake.spindex.getCommandedDegree());
            Logger.recordOutput("ABSOLUTEPOS", robot.intake.spindex.getAbsolutePos());
            Logger.recordOutput("ShootAllState", robot.getShootAllMachine().getState());
            Logger.recordOutput("ShootAllSortState", robot.getSortedShootAllMachine().getState());
            Logger.recordOutput("StartBall", robot.sortedStartBall);

        }
    }

    @Override
    public void stop() {
    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
        }
        if (currentGamepad1.b && !previousGamepad1.b && robot != null && robot.outtake != null
                && robot.outtake.vision != null) {
            Outtake.turretAimCommandOffsetDeg += -robot.outtake.vision.getTx();
        }
        getReadyShoot.update(gamepad2.b);
        toggleSorting.update(gamepad1.start || gamepad2.start);
        nextMotif.update(gamepad2.y);
        flashLights.update(gamepad2.right_bumper);
    }

    public void controlsTelemetryUpdate() {
        if (telemetryTimer.milliseconds()>200) {
            for (Control c : controls) {
                c.addTelemetry(joinedTelemetry);
            }
            if (FollowerManager.follower != null && FollowerManager.follower.getPose() != null) {
                Pose pose = FollowerManager.follower.getPose();
//                joinedTelemetry.addData("Pedro X", "%.2f", pose.getX());
//                joinedTelemetry.addData("Pedro Y", "%.2f", pose.getY());
//                joinedTelemetry.addData("Pedro H (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
            }
            joinedTelemetry.addData("Alliance", GlobalVariables.getAllianceColorName());
            joinedTelemetry.addData("Motif", GlobalVariables.getMotif());
                joinedTelemetry.addData(
                    "Loop (ms)",
                    "now %.2f | avg %.2f",
                    loopTimeTracker.getCurrentLoopTimeMs(),
                    loopTimeTracker.getTrailingAverageMs()
                );
            joinedTelemetry.addData("Use Sorting", robot.useSorting);
            joinedTelemetry.addData("Shoot Pending", shootRequestPending);
            if (robot != null && robot.outtake != null) {
                joinedTelemetry.addData("In Launch Zone", robot.outtake.isAnyPartInLaunchZone());
            }
            joinedTelemetry.addData("TXLights", robot.txLights);
            joinedTelemetry.addData("Turret Aim Offset (deg)", "%.2f", Outtake.turretAimCommandOffsetDeg);
            joinedTelemetry.update();

            telemetryTimer.reset();
        }
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    private void updateAllianceToggle() {
        if (currentGamepad2.back && !previousGamepad2.back) {
            GlobalVariables.toggleAlliance();
        }
    }

    public void stateMachinesUpdate(){
        boolean xPressed = currentGamepad1.x && !previousGamepad1.x;
        if (xPressed) {
            if (shootRequestPending) {
                // Manual override: second press while pending starts shooting immediately.
                if (!pendingShootUsesSorting && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                    robot.initShootAllMachine = true;
                    shootRequestPending = false;
                } else if (pendingShootUsesSorting && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                    robot.initSortedShootAllMachine = true;
                    shootRequestPending = false;
                }
            } else {
                if (!robot.useSorting && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                    shootRequestPending = true;
                    pendingShootUsesSorting = false;
                } else if (robot.useSorting && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                    shootRequestPending = true;
                    pendingShootUsesSorting = true;
                }
            }
        }

        if (shootRequestPending
                && robot != null
                && robot.outtake != null
                && robot.outtake.isAnyPartInLaunchZone()) {
            if (!pendingShootUsesSorting && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)) {
                robot.initShootAllMachine = true;
                shootRequestPending = false;
            } else if (pendingShootUsesSorting && sortingShootAllMachine.getState().equals(Robot.SortedShootAllStates.INIT)) {
                robot.initSortedShootAllMachine = true;
                shootRequestPending = false;
            }
        }
        shootAllMachine.update();
        sortingShootAllMachine.update();
    }

    private void applyAllianceVisionLockConfig() {
        // Keep alliance tag config in one place to avoid drift between start/loop behavior.
        if (GlobalVariables.isBlueAlliance()) {
            robot.outtake.vision.setRequiredTagId(BLUE_GOAL_TAG_ID);
        } else if (GlobalVariables.isRedAlliance()) {
            robot.outtake.vision.setRequiredTagId(RED_GOAL_TAG_ID);
        }
    }

    private void autoUpdateTurretAimOffsetWhenSettled() {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null || robot.outtake.vision == null) {
            return;
        }

        int requiredTagId = robot.outtake.vision.getRequiredTagId();
        boolean aimingGoal = robot.outtake.getAimTarget() == Outtake.AimTarget.GOAL;
        boolean requiredTagIsGoal = requiredTagId == BLUE_GOAL_TAG_ID || requiredTagId == RED_GOAL_TAG_ID;

        boolean robotStationary = isRobotMotionSettledForShot();
        boolean requiredTagVisible = robot.outtake.vision.seesTag(requiredTagId);

        boolean rearmMoved = !robotStationary;
        if (autoOffsetWaitingForMovement) {
            if (rearmMoved) {
                autoOffsetWaitingForMovement = false;
                autoOffsetStationaryTimer.reset();
            }
            return;
        }

        if (!aimingGoal || !requiredTagIsGoal) {
            autoOffsetStationaryTimer.reset();
            return;
        }

        if (robotStationary && requiredTagVisible) {
            if (autoOffsetStationaryTimer.seconds() >= autoOffsetStationarySeconds) {
                // Match manual B behavior exactly.
                Outtake.turretAimCommandOffsetDeg += -robot.outtake.vision.getTx();
                autoOffsetWaitingForMovement = true;
                autoOffsetStationaryTimer.reset();
            }
        } else {
            autoOffsetStationaryTimer.reset();
        }
    }

    private boolean isRobotMotionSettledForShot() {
        if (FollowerManager.follower == null || FollowerManager.follower.getVelocity() == null) {
            return false;
        }

        double translationalSpeedInS = Math.abs(FollowerManager.follower.getVelocity().getMagnitude());
        double angularSpeedDegS = Math.abs(Math.toDegrees(FollowerManager.follower.getAngularVelocity()));
        if (Double.isNaN(translationalSpeedInS) || Double.isInfinite(translationalSpeedInS)
                || Double.isNaN(angularSpeedDegS) || Double.isInfinite(angularSpeedDegS)) {
            return false;
        }

        return translationalSpeedInS <= autoOffsetMaxRobotSpeedInS
                && angularSpeedDegS <= autoOffsetMaxRobotAngularSpeedDegS;
    }

    private void logPsiKitData() {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null || robot.outtake.vision == null) {
            return;
        }

        double turretCmdDeg = robot.outtake.turret.getCurrentDegrees();
        double turretEncDeg = robot.outtake.turret.getEncoderDegrees();
        double turretMappedDeg = robot.outtake.turret.getMappedEncoderTurretDegrees();
        double turretMappedErrDeg = robot.outtake.turret.getMappedEncoderErrorDeg(turretCmdDeg);

        Logger.recordOutput("Turret/CmdDeg", turretCmdDeg);
        Logger.recordOutput("Turret/EncoderDeg", turretEncDeg);
        Logger.recordOutput("Turret/MappedEncoderDeg", turretMappedDeg);
        Logger.recordOutput("Turret/MappedErrorDeg", turretMappedErrDeg);
        Logger.recordOutput("Turret/EncoderVoltage", robot.outtake.turret.getEncoderVoltage());
        Logger.recordOutput("Turret/AtMinLimit", robot.outtake.turret.atMinLimit(0.0) ? 1.0 : 0.0);
        Logger.recordOutput("Turret/AtMaxLimit", robot.outtake.turret.atMaxLimit(0.0) ? 1.0 : 0.0);
        Logger.recordOutput("Turret/AimLockEnabled", robot.outtake.isAimLockEnabled() ? 1.0 : 0.0);
        Logger.recordOutput("Turret/AimTarget", robot.outtake.getAimTarget().ordinal());
        Logger.recordOutput("Turret/AimSource", robot.outtake.getActiveLockSource().ordinal());
        Logger.recordOutput("Turret/AimOffsetDeg", Outtake.turretAimCommandOffsetDeg);

        Pose followerPose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;
        if (followerPose != null) {
            Logger.recordOutput("Pinpoint/X", followerPose.getX());
            Logger.recordOutput("Pinpoint/Y", followerPose.getY());
            Logger.recordOutput("Pinpoint/HeadingDeg", Math.toDegrees(followerPose.getHeading()));
            Logger.recordOutput("Pinpoint/TotalHeadingDeg", Math.toDegrees(FollowerManager.follower.getTotalHeading()));
            Logger.recordOutput("Localization/Pinpoint/Pose2d", toPose2dFromPedroInchesAsFtcCenterRotated(followerPose));
        } else {
            Logger.recordOutput("Pinpoint/X", Double.NaN);
            Logger.recordOutput("Pinpoint/Y", Double.NaN);
            Logger.recordOutput("Pinpoint/HeadingDeg", Double.NaN);
            Logger.recordOutput("Pinpoint/TotalHeadingDeg", Double.NaN);
        }

        if (FollowerManager.follower != null && FollowerManager.follower.getVelocity() != null) {
            Logger.recordOutput("Pinpoint/VelX", FollowerManager.follower.getVelocity().getXComponent());
            Logger.recordOutput("Pinpoint/VelY", FollowerManager.follower.getVelocity().getYComponent());
            Logger.recordOutput("Pinpoint/Speed", FollowerManager.follower.getVelocity().getMagnitude());
        } else {
            Logger.recordOutput("Pinpoint/VelX", Double.NaN);
            Logger.recordOutput("Pinpoint/VelY", Double.NaN);
            Logger.recordOutput("Pinpoint/Speed", Double.NaN);
        }

        Logger.recordOutput("Limelight/HasTarget", robot.outtake.vision.hasTarget() ? 1.0 : 0.0);
        Logger.recordOutput("Limelight/TagId", robot.outtake.vision.getCurrentTagId());
        Logger.recordOutput("Limelight/Tx", robot.outtake.vision.getTx());

        LLResult latest = robot.outtake.vision.latest;
        Pose3D mt2Pose = getMt2Pose(latest);
        Pose3D mt1Pose = getMt1Pose(latest);

        Logger.recordOutput("Localization/LimelightMT2/Valid", mt2Pose != null ? 1.0 : 0.0);
        if (mt2Pose != null) {
            Logger.recordOutput("Localization/LimelightMT2/Pose2d", toPose2dFromLimelightMeters(mt2Pose));
        }

        Logger.recordOutput("Localization/LimelightMT1/Valid", mt1Pose != null ? 1.0 : 0.0);
        if (mt1Pose != null) {
            Logger.recordOutput("Localization/LimelightMT1/Pose2d", toPose2dFromLimelightMeters(mt1Pose));
        }
    }

    private Pose3D getMt2Pose(LLResult latest) {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose_MT2();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private Pose3D getMt1Pose(LLResult latest) {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            return latest.getBotpose();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private Pose2d toPose2dFromPedroInchesAsFtcCenterRotated(Pose pedroPose) {
        double pedroXIn = pedroPose.getX();
        double pedroYIn = pedroPose.getY();
        double pedroHeadingRad = pedroPose.getHeading();

        double ftcXIn = FIELD_HALF_IN - pedroYIn;
        double ftcYIn = pedroXIn - FIELD_HALF_IN;
        double ftcHeadingRad = wrapRad(pedroHeadingRad + (Math.PI * 0.5));

        return new Pose2d(
                ftcXIn * INCHES_TO_METERS,
                ftcYIn * INCHES_TO_METERS,
                Rotation2d.fromRadians(ftcHeadingRad)
        );
    }

    private Pose2d toPose2dFromLimelightMeters(Pose3D llPose) {
        double xMeters = llPose.getPosition().x;
        double yMeters = llPose.getPosition().y;
        double headingRad = Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES));
        return new Pose2d(xMeters, yMeters, Rotation2d.fromRadians(headingRad));
    }

    private static double wrapRad(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double nanosToMillis(long nanos) {
        return nanos / 1_000_000.0;
    }
}

