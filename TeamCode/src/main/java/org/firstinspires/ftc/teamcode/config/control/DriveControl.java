package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class DriveControl implements Control {

    //---------------- Software ----------------
    Drive drive;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector slowModeRE = new EdgeDetector( () -> drive.toggleSlowMode());
    EdgeDetector toggleFieldCentric = new EdgeDetector(()-> drive.toggleFieldCentric());
    EdgeDetector resetHeading = new EdgeDetector(()-> drive.resetHeading());


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


    //---------------- Interface Methods ----------------

    @Override
    public void update(){

        slowModeRE.update(gp1.dpad_down);
        resetHeading.update(gp1.start);
        toggleFieldCentric.update(gp2.back);


        if (drive.useFieldCentric){
            drive.driveFieldRelative(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x);
        } else if(drive.manualDrive){
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
            drive.setDrivePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Slow Mode?", drive.useSlowMode);
//        telemetry.addData("Use Field Centric?", drive.useFieldCentric);
        telemetry.addData("Heading", (drive.getHeading() - drive.headingOffset));
    }
}
