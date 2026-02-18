package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
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
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.OLD.GlobalVariables;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {

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

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start() {
        robot.toInit();
        shootAllMachine.start();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        updateTemporaryRequiredTagForTesting();
        controlsUpdate();
        robot.update();
        stateMachinesUpdate();
    }

    @Override
    public void stop() {

    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
            c.addTelemetry(telemetry);
        }
        telemetry.update();

        Logger.recordOutput("AbsolutePos", robot.intake.spindex.getAbsolutePos());
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

    private void updateTemporaryRequiredTagForTesting() {
        // TEMPORARY: keep tx lock on alliance goal tag until proper vision-control flow is finalized.
        if (GlobalVariables.allianceColor.equalsIgnoreCase("blue")) {
            robot.outtake.vision.setRequiredTagId(20);
        } else if (GlobalVariables.allianceColor.equalsIgnoreCase("red")) {
            robot.outtake.vision.setRequiredTagId(24);
        }
    }
}
