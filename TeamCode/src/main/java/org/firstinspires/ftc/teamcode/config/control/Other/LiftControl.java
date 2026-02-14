package org.firstinspires.ftc.teamcode.config.control.Other;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class LiftControl implements Control {

    //---------------- Software ----------------
    TemplateSubsystem template;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public LiftControl(TemplateSubsystem template, Gamepad gp1, Gamepad gp2){
        this.template = template;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public LiftControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        //this(robot.templateSystem, gp1, gp2);
        this.robot = robot;
        this.gp1 = gp1;
        this.gp2 = gp2;
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
