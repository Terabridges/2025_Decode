package org.firstinspires.ftc.teamcode.config.control.Other;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Lights;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class LightsControl implements Control {

    //---------------- Software ----------------
    Lights lights;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public LightsControl(Lights lights, Gamepad gp1, Gamepad gp2){
        this.lights = lights;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public LightsControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.other.lights, gp1, gp2);
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
