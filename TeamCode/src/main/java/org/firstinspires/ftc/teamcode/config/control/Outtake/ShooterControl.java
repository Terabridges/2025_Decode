package org.firstinspires.ftc.teamcode.config.control.Outtake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;

import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class ShooterControl implements Control {

    //---------------- Software ----------------
    Shooter shooter;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector toggleUseFlywheel = new EdgeDetector(()-> shooter.toggleUseFlywheel());
    //EdgeDetector setHoodTarget = new EdgeDetector(()-> shooter.setHoodTarget());

    //---------------- Constructor ----------------
    public ShooterControl(Shooter shooter, Gamepad gp1, Gamepad gp2){
        this.shooter = shooter;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public ShooterControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtake.shooter, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        toggleUseFlywheel.update(gp1.y);
        //setHoodTarget.update(gp1.b);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
