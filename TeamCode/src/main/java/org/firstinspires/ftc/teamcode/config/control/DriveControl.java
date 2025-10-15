package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.utility.EdgeDetector;

public class DriveControl implements Control {

    //---------------- Software ----------------
    Drive drive;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    public double FAST_MULT = 1.0;
    public double SLOW_MULT = 0.6;
    public double speed = FAST_MULT;
    EdgeDetector slowModeRE = new EdgeDetector( () -> toggleSlowMode());

    //---------------- Constructor ----------------
    public DriveControl(Drive drive, Gamepad gp1, Gamepad gp2){
        this.drive = drive;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public DriveControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.drive, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------
    public void toggleSlowMode(){
        drive.useSlowMode = !drive.useSlowMode;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void update(){

        slowModeRE.update(gp1.x);
        speed = (drive.useSlowMode ? SLOW_MULT : FAST_MULT);

        if(drive.manualDrive){
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gp1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gp1.left_stick_x;
            double yaw = gp1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            leftFrontPower *= speed;
            rightFrontPower *= speed;
            leftBackPower *= speed;
            rightBackPower *= speed;
            drive.leftFront.setPower(leftFrontPower);
            drive.rightFront.setPower(rightFrontPower);
            drive.leftBack.setPower(leftBackPower);
            drive.rightBack.setPower(rightBackPower);
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
