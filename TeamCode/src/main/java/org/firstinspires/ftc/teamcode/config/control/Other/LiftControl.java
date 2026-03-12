package org.firstinspires.ftc.teamcode.config.control.Other;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;

import org.firstinspires.ftc.teamcode.config.subsystems.Other.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class LiftControl implements Control {

    //---------------- Software ----------------
    Lift lift;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public LiftControl(Lift lift, Gamepad gp1, Gamepad gp2){
        this.lift = lift;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public LiftControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.other.lift, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        lift.moveKicker(gp2.left_stick_y);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Kicker Pos", lift.kickerCurrentPos);
        telemetry.addData("Kicker Pow", lift.speed);
    }
}
