package org.firstinspires.ftc.teamcode.config.control.Intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Clutch;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class ClutchControl implements Control {

    //---------------- Software ----------------
    Clutch clutch;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;


    //---------------- Constructor ----------------
    public ClutchControl(Clutch clutch, Gamepad gp1, Gamepad gp2){
        this.clutch = clutch;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public ClutchControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intake.clutch, gp1, gp2);
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
