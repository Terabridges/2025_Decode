package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.control.Intake.ClutchControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpindexControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpinnerControl;
import org.firstinspires.ftc.teamcode.config.control.Other.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.Other.LiftControl;
import org.firstinspires.ftc.teamcode.config.control.Other.LightsControl;
import org.firstinspires.ftc.teamcode.config.control.Other.OtherControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.OuttakeControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.TurretControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.VisionControl;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double BLUE_VISION_DIRECTION = -1.0;
    private static final double RED_VISION_DIRECTION = -1.0;
    private static final double BLUE_BIAS_MAX_DISTANCE_IN = 110.0;
    private static final double RED_BIAS_MAX_DISTANCE_IN = Double.POSITIVE_INFINITY;

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
    private JoinedTelemetry joinedTelemetry;

    public ElapsedTime loopTimer;
    public double loopTime;

    public ElapsedTime telemetryTimer;
    public double telemetryTime;

    @Override
    public void init() {
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
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        loopTimer = new ElapsedTime();
        telemetryTimer = new ElapsedTime();

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start() {
        robot.toInit();
        applyAllianceVisionLockConfig();
        robot.outtake.turret.setTxLockEnabled(true);
        shootAllMachine.start();
        loopTimer.reset();
        telemetryTimer.reset();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        applyAllianceVisionLockConfig();
        controlsUpdate();
        robot.update();
        controlsTelemetryUpdate();
        stateMachinesUpdate();
        loopTime = loopTimer.milliseconds();
        loopTimer.reset();
    }

    @Override
    public void stop() {

    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
        }
    }

    public void controlsTelemetryUpdate() {
        if (telemetryTimer.milliseconds()>200) {
            for (Control c : controls) {
                c.addTelemetry(joinedTelemetry);
            }
            joinedTelemetry.addData("Loop Time", loopTime);
            joinedTelemetry.update();

            Logger.recordOutput("AbsolutePos", robot.intake.spindex.getAbsolutePos());
            telemetryTimer.reset();
        }
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void stateMachinesUpdate(){
        if (currentGamepad1.x && !previousGamepad1.x && shootAllMachine.getState().equals(Robot.ShootAllStates.INIT)){
            robot.initShootAllMachine = true;
        }
        shootAllMachine.update();
    }

    private void applyAllianceVisionLockConfig() {
        // Keep alliance lock config in one place to avoid drift between start/loop behavior.
        double biasMagnitude = Math.abs(Turret.visionErrorBiasDeg);
        if (GlobalVariables.isBlueAlliance()) {
            robot.outtake.vision.setRequiredTagId(BLUE_GOAL_TAG_ID);
            Turret.visionDirection = BLUE_VISION_DIRECTION;
            Turret.visionErrorBiasDeg = biasMagnitude;
            Turret.visionBiasMaxDistanceIn = BLUE_BIAS_MAX_DISTANCE_IN;
        } else if (GlobalVariables.isRedAlliance()) {
            robot.outtake.vision.setRequiredTagId(RED_GOAL_TAG_ID);
            Turret.visionDirection = RED_VISION_DIRECTION;
            Turret.visionErrorBiasDeg = biasMagnitude;
            Turret.visionBiasMaxDistanceIn = RED_BIAS_MAX_DISTANCE_IN;
        }
    }
}
