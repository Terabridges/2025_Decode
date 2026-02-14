package org.firstinspires.ftc.teamcode.config.control.Other;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Other;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class OtherControl implements Control {

    //---------------- Software ----------------
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    Other other;

    //---------------- Constructor ----------------
    public OtherControl(Other other, Gamepad gp1, Gamepad gp2){
        this.other = other;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public OtherControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.other, gp1, gp2);
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
