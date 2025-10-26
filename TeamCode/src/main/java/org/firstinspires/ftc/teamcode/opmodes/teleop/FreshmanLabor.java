package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

//TODO make sure hood angle is consistent, starts at the same place

@TeleOp(name="FreshmanLabor", group="TeleOp")
public class FreshmanLabor extends LinearOpMode {

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    public Gamepad currentGamepad2;
    public Gamepad previousGamepad2;

    double targetRPM = 0;
    double rpmIncrement = 1000;
    boolean runShooter = false;
    double targetAngle = 0;
    double angleIncrement = 0.2;

    @Override
    public void runOpMode(){

        Robot robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();


        waitForStart();

        robot.toInit();

        while (opModeIsActive()){

            //Left Stick X controls spindex
            if (gamepad1.left_stick_x > 0.1){
                robot.transfer.spindexRight();
            } else if (gamepad1.left_stick_x < -0.1){
                robot.transfer.spindexLeft();
            } else {
                robot.transfer.spindexZero();
            }

            //Right stick Y controls spinner
            if (gamepad1.right_stick_y < -0.1){
                robot.intake.spinnerIn();
            } else if (gamepad1.right_stick_y > 0.1){
                robot.intake.spinnerOut();
            } else {
                robot.intake.spinnerZero();
            }

            //back controls clutch
            if (currentGamepad1.back && !previousGamepad1.back){
                robot.transfer.toggleClutch();
            }

            //Dpad up and down to change RPM
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                targetRPM += rpmIncrement;
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                targetRPM -= rpmIncrement;
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
                runShooter = !runShooter;
            }
            robot.shooter.setShooterRPM(targetRPM);

            //right bumper sets angle
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                robot.shooter.setHoodPos(targetAngle);
            }

            //start toggles turret lock
            if(currentGamepad1.start && !previousGamepad1.start){
                robot.shooter.toggleTurretLock();
            }

            //telemetry
            telemetry.addData("Distance", robot.vision.getDistanceInches());
            telemetry.addData("Current RPM", robot.shooter.getShooterRPM());
            telemetry.addData("Current Angle", robot.shooter.getHoodPos());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("RPM Increment", rpmIncrement);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Angle Increment", angleIncrement);
            telemetry.addData("Turret Lock?", robot.shooter.useTurretLock);
            telemetry.addData("Clutch engaged?", robot.transfer.isClutchDown);
            telemetry.update();
        }
    }
}
