package org.firstinspires.ftc.teamcode.config.teleControl;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

public class DriveControl implements Control{

    //Software
    Drive drive;
    public Robot robot;
    public Gamepad gp1;
    public Gamepad gp2;
    public double speed = 1.0;

    //Constructor
    public DriveControl(Drive drive, Gamepad gp1, Gamepad gp2) {
        this.drive = drive;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public DriveControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.drive, gp1, gp2);
        this.robot = robot;
    }

    //Methods

    //Interface Methods
    @Override
    public void update(){
        double max;
        double axial = -gp1.left_stick_y;
        double lateral = gp1.left_stick_x;
        double yaw = gp1.right_stick_x;
        double leftFrontPower = axial + lateral + (yaw);
        double rightFrontPower = axial - lateral - (yaw);
        double leftBackPower = axial - lateral + (yaw);
        double rightBackPower = axial + lateral - (yaw);
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

    @Override
    public void addTelemetry(Telemetry telemetry){
    }
}