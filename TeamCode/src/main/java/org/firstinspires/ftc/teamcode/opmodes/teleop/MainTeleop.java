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
        SPIN0,
        CLUTCHDOWN1,
        CLUTCHDOWNFAR1,
        SPIN1,
        CLUTCHDOWN2,
        CLUTCHDOWNFAR2,
        SPIN2,
        CLUTCHDOWN3,
        CLUTCHDOWNFAR3,
    }

    public enum clutchStates {
        INIT,
        SPIN,
        CLUTCHDOWN,
        CLUTCHUP
    }

    public double clutchDownTime = 0.15;
    public double clutchDownFarTime = 0.72;

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
                .transition(()->(currentGamepad1.x && !previousGamepad1.x), shootStates.SPIN0)

                .state(shootStates.SPIN0)
                .onEnter(()-> {
                    transfer.ballRightSmall();
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.CLUTCHDOWN1)
                .transitionTimed(1.5, shootStates.CLUTCHDOWN1)

                .state(shootStates.CLUTCHDOWN1)
                .onEnter(()-> transfer.setClutchDown())
                .transitionTimed(clutchDownTime, shootStates.CLUTCHDOWNFAR1)

                .state(shootStates.CLUTCHDOWNFAR1)
                .onEnter(()-> transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN1)
                .onExit(()-> {
                    transfer.setClutchUp();
                })

                .state(shootStates.SPIN1)
                .onEnter(()-> {
                    transfer.ballRight();
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.CLUTCHDOWN2)
                .transitionTimed(1.5, shootStates.CLUTCHDOWN2)

                .state(shootStates.CLUTCHDOWN2)
                .onEnter(()-> {
                    transfer.setClutchDown();
                })
                .transitionTimed(clutchDownTime, shootStates.CLUTCHDOWNFAR2)

                .state(shootStates.CLUTCHDOWNFAR2)
                .onEnter(()-> transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN2)
                .onExit(()-> transfer.setClutchUp())

                .state(shootStates.SPIN2)
                .onEnter(()-> {
                    transfer.ballRight();
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.CLUTCHDOWN3)
                .transitionTimed(1.5, shootStates.CLUTCHDOWN3)

                .state(shootStates.CLUTCHDOWN3)
                .onEnter(()-> {
                    transfer.setClutchDown();
                })
                .transitionTimed(clutchDownTime, shootStates.CLUTCHDOWNFAR3)

                .state(shootStates.CLUTCHDOWNFAR3)
                .onEnter(()-> transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.INIT)
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
        Shooter shooter = robot.shooter;
        return new StateMachineBuilder()
                .state(clutchStates.INIT)
                .transition(()->(currentGamepad1.a && !previousGamepad1.a), clutchStates.SPIN)

                .state(clutchStates.SPIN)
                .onEnter(()-> transfer.ballRightSmall())
                .transition(()->transfer.spindexAtTarget(), clutchStates.CLUTCHDOWN)
                .transitionTimed(1.5, clutchStates.CLUTCHDOWN)

                .state(clutchStates.CLUTCHDOWN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.setClutchDown();
                    transfer.isDetecting = false;
                })
                .transitionTimed(clutchDownTime, clutchStates.CLUTCHUP)
                .onExit(()-> transfer.setClutchDownFar())

                .state(clutchStates.CLUTCHUP)
                .transitionTimed(clutchDownFarTime, clutchStates.INIT)
                .onExit(()->{
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
}
