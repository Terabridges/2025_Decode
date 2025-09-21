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

    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
