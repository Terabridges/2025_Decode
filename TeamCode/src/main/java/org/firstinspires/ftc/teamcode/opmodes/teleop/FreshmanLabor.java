package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;

//TODO make sure hood angle is consistent, starts at the same place

@TeleOp(name="FreshmanLabor", group="TeleOp")
public class FreshmanLabor extends LinearOpMode {

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    private JoinedTelemetry joinedTelemetry;

    double rpmIncrement = 1000;
    double targetAngle = 0;
    double angleIncrement = 0.2;

    public double clutchDownTime = 0.15;
    public double clutchDownFarTime = 0.6;

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

    public StateMachine shootAllMachine;
    public StateMachine clutchSuperMachine;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        Robot robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        shootAllMachine = getShootAllMachine(robot);
        clutchSuperMachine = getClutchSuperMachine(robot);

        robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        robot.toInit();
        shootAllMachine.start();
        clutchSuperMachine.start();
        robot.shooter.shooterShoot = false;
        robot.shooter.manualTurret = true;
        robot.transfer.useSpindexPID = true;
        robot.transfer.isDetecting = false;
        robot.shooter.useData = false;

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            clutchSuperMachine.update();

            //Left Stick X controls spindex
            if (currentGamepad1.left_stick_x > 0.1 && previousGamepad1.left_stick_x == 0){
                robot.transfer.ballRight();
            } else if (currentGamepad1.left_stick_x < -0.1 && previousGamepad1.left_stick_x == 0){
                robot.transfer.ballLeft();
            }

            //Right stick Y controls spinner
            if (currentGamepad1.right_stick_y < -0.1){
                robot.intake.spinnerIn();
            } else {
                robot.intake.spinnerZero();
            }

            //Dpad up and down to change RPM
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                robot.shooter.targetRPM += rpmIncrement;
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                robot.shooter.targetRPM -= rpmIncrement;
            }

            //Dpad left and right to change RPM incrememt
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                if(rpmIncrement == 50){
                    rpmIncrement = 100;
                } else if(rpmIncrement == 100){
                    rpmIncrement = 500;
                } else if(rpmIncrement == 500){
                    rpmIncrement = 1000;
                } else if(rpmIncrement == 1000){
                    rpmIncrement = 50;
                }
            }

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                if(rpmIncrement == 1000){
                    rpmIncrement = 500;
                } else if(rpmIncrement == 500){
                    rpmIncrement = 100;
                } else if(rpmIncrement == 100){
                    rpmIncrement = 50;
                } else if(rpmIncrement == 50){
                    rpmIncrement = 1000;
                }
            }

            //A and Y to change angle
            if (currentGamepad1.y && !previousGamepad1.y){
                targetAngle += angleIncrement;
                if (targetAngle > 1){
                    targetAngle -= 1;
                }
            }

            if (currentGamepad1.a && !previousGamepad1.a){
                targetAngle -= angleIncrement;
                if (targetAngle < 0){
                    targetAngle += 1;
                }
            }

            //X and B to change angle increment
            if(currentGamepad1.b && !previousGamepad1.b){
                if(angleIncrement == 0.025){
                    angleIncrement = 0.05;
                } else if(angleIncrement == 0.05){
                    angleIncrement = 0.1;
                } else if(angleIncrement == 0.1){
                    angleIncrement = 0.2;
                } else if(angleIncrement == 0.2){
                    angleIncrement = 0.025;
                }
            }

            if(currentGamepad1.x && !previousGamepad1.x){
                if(angleIncrement == 0.2){
                    angleIncrement = 0.1;
                } else if(angleIncrement == 0.1){
                    angleIncrement = 0.05;
                } else if(angleIncrement == 0.05){
                    angleIncrement = 0.025;
                } else if(angleIncrement == 0.025){
                    angleIncrement = 0.2;
                }
            }

            //left bumper controls shooter on
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                robot.shooter.toggleShooter();
            }

            //right bumper sets angle
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                robot.shooter.hood.setPosition(targetAngle);
            }

            //start toggles turret lock
            if(currentGamepad1.start && !previousGamepad1.start){
                robot.shooter.toggleTurretLock();
            }

            //back controls clutch
            if (currentGamepad1.back && !previousGamepad1.back){
                robot.transfer.toggleClutch();
            }

            //rstickButton controls shoot all macro

            //lstickButton controls shoot one macro

            robot.update();

            //telemetry
            joinedTelemetry.addData("Distance", robot.vision.getDistanceInches());
            joinedTelemetry.addData("Current RPM", robot.shooter.getShooterRPM());
            joinedTelemetry.addData("Current Angle", robot.shooter.getHoodPos());
            joinedTelemetry.addData("Target RPM", robot.shooter.targetRPM);
            joinedTelemetry.addData("RPM Increment", rpmIncrement);
            joinedTelemetry.addData("Target Angle", targetAngle);
            joinedTelemetry.addData("Angle Increment", angleIncrement);
            joinedTelemetry.addData("Locked on?", robot.shooter.useTurretLock);
            joinedTelemetry.update();
        }
    }

    public StateMachine getShootAllMachine (Robot robot){
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(shootStates.INIT)
                .transition(()->(currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button), shootStates.SPIN0)

                .state(shootStates.SPIN0)
                .onEnter(()-> {
                    transfer.ballRightSmall();
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.CLUTCHDOWN1)

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
                    transfer.ballLeftSmall();
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
                .transition(()->(currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button), clutchStates.SPIN)

                .state(clutchStates.SPIN)
                .onEnter(()-> transfer.ballRightSmall())
                .transition(()->transfer.spindexAtTarget(), clutchStates.CLUTCHDOWN)

                .state(clutchStates.CLUTCHDOWN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    transfer.setClutchDown();
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
                })

                .build();
    }
}
