package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.control.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.TransferControl;
import org.firstinspires.ftc.teamcode.config.control.VisionControl;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.wrappers.MotorWrapper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.io.PrintWriter;
import java.io.StringWriter;

@TeleOp(name = "MainTeleOp PsiKit", group = "TeleOp")
public class MainTeleopPsikit extends LinearOpMode {

    private final FtcLoggingSession psiKit = new FtcLoggingSession();

    private Robot robot;

    public DriveControl driveControl;
    public IntakeControl intakeControl;
    public ShooterControl shooterControl;
    public TransferControl transferControl;
    public VisionControl visionControl;
    public List<Control> controls;

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    public Gamepad currentGamepad2;
    public Gamepad previousGamepad2;

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

    public enum clutchStates {
        INIT,
        SPIN,
        CLUTCHDOWN,
        CLUTCHUP
    }

    public enum resetStates {
        INIT,
        SPIN,
        RESET,
        SPIN2,
        RESET2
    }

    public double clutchDownTime = 0.1;
    public double clutchDownFarTime = 0.4;
    public double spinTime = 2.75;
    public double spinUpTimeout = 1.75;
    int fromRed = -90;
    private boolean turretAimAssist = false;
    private boolean holdingForShoot = false;

    public StateMachine shootAllMachine;
    public StateMachine clutchSuperMachine;
    public StateMachine resetMachine;

    @Override
    public void runOpMode() {
        try {
            // Start PsiKit first so the OpMode's hardwareMap (and gamepads) are wrapped.
            // AdvantageScope defaults to port 5800. Set to 0 to disable the server.
            psiKit.start(this, 5802);

            // Pinpoint logging model: "normal hardware".
            // PedroPathing owns pinpoint.update() via follower.update(); PsiKit only logs Pinpoint
            // if user code accessed it via hardwareMap.get(...).
            psiKit.enablePinpointOdometryLogging = false;

            // Fastest practical logging profile:
            // - Avoids per-loop non-bulk motor readbacks (power/mode/etc.) while still logging
            //   command-side values (safe for replay).
            MotorWrapper.logProfile = MotorWrapper.LOG_PROFILE_FAST;

            // Global non-bulk sampling throttle (IMU / I2C sensors / ADC-like reads).
            // Skips reads and table writes in between; LogTable retains the last value.
            FtcLogTuning.nonBulkReadPeriodSec = 0.05; // .25 is 4 Hz

            // For A/B timing tests: disable expensive sensors in PsiKit's background polling.
            // - IMU is typically not needed when using Pinpoint for heading/pose.
            // - Color sensor background polling can be expensive; if your robot code actually
            //   uses it, PsiKit will still read on-demand via passthrough.
            FtcLogTuning.logImu = false;
            FtcLogTuning.processColorDistanceSensorsInBackground = false;

            // Motor current reads are typically expensive (non-bulk Lynx transactions).
            // Log them on a slow tier.
            FtcLogTuning.logMotorCurrent = false;
            FtcLogTuning.motorCurrentReadPeriodSec = 0.10; // 10 Hz

            // Prefetch Lynx bulk data once per hub each loop so the "first read" tax
            // shows up under PsiKit/logTimes (us)/BulkPrefetch/... instead of a motor.
            FtcLogTuning.prefetchBulkDataEachLoop = true;

            // Pinpoint is a large fixed-cost I2C bulk read; throttle it to reduce FullCycleMS.
            // NOTE: if your loop is ~50ms, a 20ms period won't skip anything. Use > loop period.
            // Resilient across device/firmware: if minimal scope isn't supported, it's ignored.
            FtcLogTuning.pinpointReadPeriodSec = 0.10; // 10 Hz
            FtcLogTuning.pinpointLoggerCallsUpdate = false; // Pedro follower already calls pinpoint.update()
            FtcLogTuning.pinpointUseMinimalBulkReadScope = true;

            // Only the flywheel motors need encoder velocity; drivetrain motors don't.
            MotorWrapper.setVelocityLoggedMotors("fly_left", "fly_right");

            // Velocity reads can be expensive; sampling at ~50Hz is usually sufficient for logs.
            MotorWrapper.velocityRefreshPeriodSec = 0.02;

            // Drivetrain encoders are not used; skip encoder-related reads to reduce loop/log time.
                MotorWrapper.setEncoderSkippedMotors(
                    "left_front", "left_back", "right_front", "right_back",
                    "spinner"
                );

            try {
                robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
                FollowerManager.getFollower(hardwareMap, new Pose());
            } catch (Throwable t) {
                // In JVM replay runs, we may not be able to construct the full robot because
                // some hardware devices don't have mock wrappers available. Keep replay usable
                // by falling back to a minimal loop that still logs/replays DS + OpModeControls.
                if (Logger.isReplay()) {
                    // Record the exception later from inside the replay-only loop. In practice,
                    // Logger.recordOutput can throw here if the logging session isn't fully running yet.
                    runReplayOnlyLoop(t);
                    return;
                }
                throw t;
            }

            driveControl = new DriveControl(robot, gamepad1, gamepad2);
            intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
            shooterControl = new ShooterControl(robot, gamepad1, gamepad2);
            transferControl = new TransferControl(robot, gamepad1, gamepad2);
            visionControl = new VisionControl(robot, gamepad1, gamepad2);

            controls = new ArrayList<>(Arrays.asList(
                    intakeControl,
                    shooterControl,
                    transferControl,
                    driveControl,
                    visionControl
            ));

            currentGamepad1 = new Gamepad();
            previousGamepad1 = new Gamepad();

            currentGamepad2 = new Gamepad();
            previousGamepad2 = new Gamepad();

            shootAllMachine = getShootAllMachine(robot);
            clutchSuperMachine = getClutchSuperMachine(robot);
            resetMachine = getSpindexResetMachine(robot);

            robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            while (opModeInInit()) {
                double beforeUserStart = Logger.getRealTimestamp();
                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/LoggerPeriodicBeforeUser")) {
                    Logger.periodicBeforeUser();
                }
                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/PsiKitLogOncePerLoop")) {
                    psiKit.logOncePerLoop(this);
                }
                double beforeUserEnd = Logger.getRealTimestamp();

                gamepadUpdate();

                if (currentGamepad1.a && !previousGamepad1.a) {
                    if(GlobalVariables.motif.equals("PPG")){
                        GlobalVariables.motif = "GPP";
                    } else if(GlobalVariables.motif.equals("GPP")){
                        GlobalVariables.motif = "PGP";
                    } else if(GlobalVariables.motif.equals("PGP")){
                        GlobalVariables.motif = "PPG";
                    }
                }

                if (currentGamepad1.b && !previousGamepad1.b){
                    if(GlobalVariables.allianceColor.equals("red")){
                        GlobalVariables.allianceColor = "blue";
                    } else if (GlobalVariables.allianceColor.equals("blue")){
                        GlobalVariables.allianceColor = "red";
                    }
                }

                telemetry.addData("Press A to change Motif. Press B to change alliance color.", "");
                telemetry.addData("Motif", GlobalVariables.motif);
                telemetry.addData("Alliance Color", GlobalVariables.allianceColor);
                telemetry.update();

                double afterUserStart = Logger.getRealTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );

                idle();
            }

            waitForStart();

            robot.toInit();
            shootAllMachine.start();
            clutchSuperMachine.start();
            resetMachine.start();

            while (opModeIsActive()) {
                double beforeUserStart = Logger.getRealTimestamp();
                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/LoggerPeriodicBeforeUser")) {
                    Logger.periodicBeforeUser();
                }
                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/PsiKitLogOncePerLoop")) {
                    psiKit.logOncePerLoop(this);
                }
                double beforeUserEnd = Logger.getRealTimestamp();

                // Copy gamepad state after PsiKit has applied replay controls (if replaying).
                gamepadUpdate();

                // If driver moves sticks, give control to driver; otherwise let Pedro run if busy.
                boolean driverActive = Math.abs(currentGamepad1.left_stick_x) > 0.1
                        || Math.abs(currentGamepad1.left_stick_y) > 0.1
                        || Math.abs(currentGamepad1.right_stick_x) > 0.1;
                boolean followerBusy = follower.isBusy();
                boolean shooting = shootAllMachine.getState() != shootStates.INIT;
                boolean holdDuringShoot = shooting && !driverActive;

                if (holdDuringShoot && !holdingForShoot) {
                    // Freeze pose using follower hold instead of zeroing power.
                    follower.holdPoint(follower.getPose());
                    holdingForShoot = true;
                } else if ((!shooting || driverActive) && holdingForShoot) {
                    follower.breakFollowing();
                    holdingForShoot = false;
                }

                robot.drive.manualDrive = driverActive || (!followerBusy && !holdingForShoot);
                // Always update to keep pose fresh; manual drive will overwrite any motor commands when active.
                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/FollowerUpdate")) {
                    follower.update();
                }

                // Toggle turret auto-aim with GP1 dpad_up
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    turretAimAssist = !turretAimAssist;
                    if (!turretAimAssist) {
                        robot.shooter.useTurretLock = false;
                        robot.shooter.turretLockController.reset();
                    }
                }
                if (turretAimAssist) {
                    updateTurretAimTeleop(robot);
                }

                // Quick reset: GP2 B seeds follower pose to field center facing goals.
                if (currentGamepad2.b && !previousGamepad2.b) {
                    follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
                }

                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/RobotUpdate")) {
                    robot.update();
                }

                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/ControlsUpdate")) {
                    controlsUpdate();
                }

                try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/StateMachinesUpdate")) {
                    stateMachinesUpdate();
                }

                // Stop everything
                if(currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button){
                    shootAllMachine.setState(shootStates.INIT);
                    clutchSuperMachine.setState(clutchStates.INIT);
                    resetMachine.setState(resetStates.INIT);
                    robot.intake.spinnerMacroTarget = 0;
                    robot.shooter.shooterShoot = false;
                    robot.transfer.isDetecting = true;
                    robot.transfer.emptyBalls();
                    robot.intake.spinnerMacro = false;
                    robot.transfer.max = 0.4;
                }

                //Switch Alliance
                if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                    if(GlobalVariables.allianceColor.equals("red")){
                        GlobalVariables.allianceColor = "blue";
                    } else if (GlobalVariables.allianceColor.equals("blue")){
                        GlobalVariables.allianceColor = "red";
                    }
                }

                //Switch Motif
                if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                    if(GlobalVariables.motif.equals("PPG")){
                        GlobalVariables.motif = "GPP";
                    } else if(GlobalVariables.motif.equals("GPP")){
                        GlobalVariables.motif = "PGP";
                    } else if(GlobalVariables.motif.equals("PGP")){
                        GlobalVariables.motif = "PPG";
                    }
                }

                double afterUserStart = Logger.getRealTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );

                idle();
            }
        } finally {
            psiKit.end();
        }
    }

    private void runReplayOnlyLoop(Throwable replayInitFailure) {
        int loopCount = 0;
        boolean recordedInitFailure = false;

        while (opModeInInit()) {
            double beforeUserStart = Logger.getRealTimestamp();
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/LoggerPeriodicBeforeUser")) {
                Logger.periodicBeforeUser();
            }

            if (!recordedInitFailure && replayInitFailure != null) {
                recordedInitFailure = true;
                recordReplayInitFailure(replayInitFailure);
            }

            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/PsiKitLogOncePerLoop")) {
                psiKit.logOncePerLoop(this);
            }
            double beforeUserEnd = Logger.getRealTimestamp();

            telemetry.addLine("Replay-only mode (hardware unavailable in JVM).");
            telemetry.addLine("PsiKit is replaying OpModeControls + DS inputs.");
            telemetry.update();

                double afterUserStart = Logger.getRealTimestamp();
            Logger.periodicAfterUser(
                    afterUserStart - beforeUserEnd,
                    beforeUserEnd - beforeUserStart
            );

            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            double beforeUserStart = Logger.getRealTimestamp();
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/LoggerPeriodicBeforeUser")) {
                Logger.periodicBeforeUser();
            }

            if (!recordedInitFailure && replayInitFailure != null) {
                recordedInitFailure = true;
                recordReplayInitFailure(replayInitFailure);
            }

            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/LogPeriodicBreakdownMS/PsiKitLogOncePerLoop")) {
                psiKit.logOncePerLoop(this);
            }
            double beforeUserEnd = Logger.getRealTimestamp();

            Logger.recordOutput("ReplayOnly/LoopCount", loopCount++);

                double afterUserStart = Logger.getRealTimestamp();
            Logger.periodicAfterUser(
                    afterUserStart - beforeUserEnd,
                    beforeUserEnd - beforeUserStart
            );

            idle();
        }
    }

    private static void recordReplayInitFailure(Throwable t) {
        try {
            // Numeric flag so we can verify via summary tooling.
            Logger.recordOutput("LoggedRobot/ReplayInitExceptionPresent", 1);
            Logger.recordOutput("LoggedRobot/ReplayInitExceptionType", t.getClass().getName());
            Logger.recordOutput("LoggedRobot/ReplayInitExceptionMessage", String.valueOf(t.getMessage()));
            Logger.recordOutput("LoggedRobot/ReplayInitExceptionStack", stackTraceSummary(t));
        } catch (Throwable ignored) {
            // Best-effort logging; never block replay.
        }
    }

    private static String stackTraceSummary(Throwable t) {
        try {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            t.printStackTrace(pw);
            pw.flush();
            String s = sw.toString();
            final int maxChars = 8000;
            if (s.length() > maxChars) {
                return s.substring(0, maxChars) + "\n... (truncated)";
            }
            return s;
        } catch (Throwable ignored) {
            return t.toString();
        }
    }
    public void controlsUpdate() {
        for (Control c : controls) {
            String name = c.getClass().getSimpleName();

            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/ControlsUpdate/" + name + "/update")) {
                c.update();
            }

            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/ControlsUpdate/" + name + "/addTelemetry")) {
                c.addTelemetry(telemetry);
            }
        }
        telemetry.addData("Shooting State", shootAllMachine.getState());
        telemetry.addData("Turret Auto Aim", turretAimAssist);

        try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/ControlsUpdate/telemetryUpdate")) {
            telemetry.update();
        }
    }

    public void gamepadUpdate() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void stateMachinesUpdate() {
        shootAllMachine.update();
        clutchSuperMachine.update();
        resetMachine.update();
    }

    // only Counterclockwise rotation
    public StateMachine getShootAllMachine(Robot robot) {
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(shootStates.INIT)
                .transition(() -> (currentGamepad1.x && !previousGamepad1.x), shootStates.PRESPIN)

                .state(shootStates.PRESPIN)
                .onEnter(() -> {
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                    if (transfer.desiredRotate == 1) {
                        transfer.ballLeft();
                    } else if (transfer.desiredRotate == 2) {
                        transfer.ballRight();
                    }
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN)

                .state(shootStates.CLUTCHDOWN)
                .onEnter(() -> {
                    transfer.max = 0.275;
                    transfer.setClutchBarelyDown();
                })
                .transitionTimed(clutchDownTime, shootStates.WAIT1)

                .state(shootStates.WAIT1)
                .transition(() -> shooter.isFarShot(), shootStates.SPIN1)
                .transition(() -> !shooter.isFarShot(), shootStates.SPIN)

                .state(shootStates.SPIN)
                .onEnter(() -> {
                    transfer.max = 0.2;
                    transfer.ballLeft();
                    transfer.ballLeft();
                })
                .transition(() -> transfer.spindexAtTarget(), shootStates.SPIN3)
                .transitionTimed(spinTime, shootStates.SPIN3)

                .state(shootStates.SPIN1)
                .onEnter(() -> {
                    transfer.ballLeftSmall();
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN1)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN1)

                .state(shootStates.CLUTCHDOWN1)
                .onEnter(() -> transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN2)
                .onExit(() -> transfer.setClutchBarelyDown())

                .state(shootStates.SPIN2)
                .onEnter(() -> {
                    transfer.ballLeft();
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN2)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN2)

                .state(shootStates.CLUTCHDOWN2)
                .onEnter(() -> transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN3)
                .onExit(() -> {
                    transfer.setClutchBarelyDown();
                    transfer.ballRightSmall();
                })

                .state(shootStates.SPIN3)
                .onEnter(() -> {
                    transfer.max = 0.275;
                    transfer.ballLeftSmall();
                    transfer.ballLeft();
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.WAIT2)
                .transitionTimed(spinUpTimeout, shootStates.WAIT2)
                .onExit(() -> transfer.setClutchDownFar())

                .state(shootStates.WAIT2)
                .transitionTimed(clutchDownFarTime, shootStates.INIT)
                .onExit(() -> {
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

    public StateMachine getClutchSuperMachine(Robot robot) {
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        Shooter shooter = robot.shooter;
        return new StateMachineBuilder()
                .state(clutchStates.INIT)
                .transition(() -> (currentGamepad1.a && !previousGamepad1.a), clutchStates.SPIN)

                .state(clutchStates.SPIN)
                .onEnter(() -> {
                    transfer.ballRightSmall();
                    shooter.shooterShoot = true;
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    transfer.isDetecting = false;
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), clutchStates.CLUTCHDOWN)
                .transitionTimed(spinUpTimeout, clutchStates.CLUTCHDOWN)

                .state(clutchStates.CLUTCHDOWN)
                .onEnter(() -> {
                    transfer.setClutchDown();
                })
                .transitionTimed(clutchDownTime, clutchStates.CLUTCHUP)
                .onExit(() -> transfer.setClutchDownFar())

                .state(clutchStates.CLUTCHUP)
                .transitionTimed(clutchDownFarTime, clutchStates.INIT)
                .onExit(() -> {
                    intake.spinnerMacro = false;
                    intake.spinnerMacroTarget = 0;
                    transfer.setClutchUp();
                    transfer.ballLeftSmall();
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.ballList[1] = "E";
                })

                .build();
    }

    public StateMachine getSpindexResetMachine(Robot robot) {
        Transfer transfer = robot.transfer;
        return new StateMachineBuilder()
                .state(resetStates.INIT)
                .transition(() -> (currentGamepad1.b && !previousGamepad1.b), resetStates.SPIN)

                .state(resetStates.SPIN)
                .onEnter(() -> {
                    transfer.useSpindexPID = false;
                    transfer.isDetecting = false;
                    transfer.spindexManualPow = 0.075;
                    transfer.spindexTarget = 0;
                })
                .transition(() -> transfer.isRed, resetStates.RESET)

                .state(resetStates.RESET)
                .onEnter(() -> {
                    transfer.spindexManualPow = 0;
                    transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    transfer.useSpindexPID = true;
                })
                .transitionTimed(0.1, resetStates.SPIN2)
                .onExit(() -> transfer.spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER))

                .state(resetStates.SPIN2)
                .onEnter(() -> transfer.spindexTarget += fromRed)
                .transition(() -> transfer.spindexAtTarget(), resetStates.RESET2)

                .state(resetStates.RESET2)
                .onEnter(() -> {
                    transfer.spindexTarget = 0;
                    transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .transitionTimed(0.1, resetStates.INIT)
                .onExit(() -> {
                    transfer.spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    transfer.isDetecting = true;
                })

                .build();
    }

    /** TeleOp aim logic: lock when vision has the target, otherwise coarse aim at the goal. */
    private void updateTurretAimTeleop(Robot robot) {
        if (robot == null || robot.shooter == null || follower == null) return;

        if (robot.shooter.hasDesiredTarget) {
            robot.shooter.useTurretLock = true;
        } else {
            robot.shooter.useTurretLock = false;
            robot.shooter.turretLockController.reset();
            coarseTurretAimAtGoalTeleop(robot);
        }
    }

    private void coarseTurretAimAtGoalTeleop(Robot robot) {
        Pose robotPose = follower.getPose();
        Pose target = getGoalPoseTeleop();
        if (robotPose == null || target == null) return;
        robot.shooter.aimTurretAtFieldPose(
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getHeading(),
                target.getX(),
                target.getY()
        );
    }

    private Pose getGoalPoseTeleop() {
        boolean blue = GlobalVariables.allianceColor.equalsIgnoreCase("blue");
        return blue
                ? new Pose(0, 144, Math.toRadians(90))
                : new Pose(144, 144, Math.toRadians(90));
    }
}
