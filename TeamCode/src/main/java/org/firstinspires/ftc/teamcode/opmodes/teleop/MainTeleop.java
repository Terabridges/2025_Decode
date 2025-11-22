package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.control.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.TransferControl;
import org.firstinspires.ftc.teamcode.config.control.VisionControl;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleop extends LinearOpMode {

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
        CLUTCHDOWN,
        SPIN,
        CLUTCHDOWNFAR
    }

    public enum clutchStates {
        INIT,
        CLUTCHDOWN,
        CLUTCHUP
    }

    public StateMachine shootAllMachine;
    public StateMachine clutchSuperMachine;

    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        driveControl = new DriveControl(robot, gamepad1, gamepad2);
        intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
        shooterControl = new ShooterControl(robot, gamepad1, gamepad2);
        transferControl = new TransferControl(robot, gamepad1, gamepad2);
        visionControl = new VisionControl(robot, gamepad1, gamepad2);

        controls = new ArrayList<>(Arrays.asList(intakeControl, shooterControl, transferControl, driveControl, visionControl));

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        shootAllMachine = getShootAllMachine(robot);
        clutchSuperMachine = getClutchSuperMachine(robot);

        robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        robot.toInit();
        shootAllMachine.start();
        clutchSuperMachine.start();

        while (opModeIsActive()){
            gamepadUpdate();
            robot.update();
            controlsUpdate();
            stateMachinesUpdate();
        }
    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
            c.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void stateMachinesUpdate(){
        shootAllMachine.update();
        clutchSuperMachine.update();
    }

    public StateMachine getShootAllMachine (Robot robot){
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(shootStates.INIT)
                .transition(()->(currentGamepad1.x && !previousGamepad1.x), shootStates.CLUTCHDOWN)

                .state(shootStates.CLUTCHDOWN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    transfer.setClutchDown();
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                })
                .transitionTimed(0.5, shootStates.SPIN)

                .state(shootStates.SPIN)
                .onEnter(()-> {
                    transfer.ballRight();
                    transfer.ballRight();
                    transfer.ballRightSmall();
                })
                .transitionTimed(3.2, shootStates.CLUTCHDOWNFAR)

                .state(shootStates.CLUTCHDOWNFAR)
                .onEnter(()-> {
                    transfer.setClutchDownFar();
                })
                .transitionTimed(1.5, shootStates.INIT)
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
                .state(clutchStates.INIT)
                .transition(()->(currentGamepad1.a && !previousGamepad1.a), clutchStates.CLUTCHDOWN)

                .state(clutchStates.CLUTCHDOWN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    transfer.setClutchDownFar();
                })
                .transitionTimed(1.2, clutchStates.CLUTCHUP)

                .state(clutchStates.CLUTCHUP)
                .onEnter(()->{
                    intake.spinnerMacro = false;
                    intake.spinnerMacroTarget = 0;
                    transfer.setClutchUp();
                })
                .transitionTimed(0.2, clutchStates.INIT)

                .build();
    }
}
