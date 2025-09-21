package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.utility.EdgeDetector;

public class TemplateControl implements Control {

    //---------------- Software ----------------
    TemplateSubsystem template;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector setServoRE = new EdgeDetector(() -> template.setServoOn());

    //---------------- Constructor ----------------
    public TemplateControl(TemplateSubsystem template, Gamepad gp1, Gamepad gp2){
        this.template = template;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public TemplateControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        //this(robot.templateSystem, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        setServoRE.update(gp1.a);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
