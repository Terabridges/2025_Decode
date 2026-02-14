package org.firstinspires.ftc.teamcode.config.control.OLD;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Vision;
import org.firstinspires.ftc.teamcode.config.utility.OLD.GlobalVariables;

public class VisionControl implements Control {

    //---------------- Software ----------------
    Vision vision;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public VisionControl(Vision vision, Gamepad gp1, Gamepad gp2){
        this.vision = vision;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public VisionControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.vision, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Vision Error", vision.getTx());
        telemetry.addData("Distance", vision.getDistanceInches());
        telemetry.addData("Alliance Color", GlobalVariables.allianceColor);
    }
}
