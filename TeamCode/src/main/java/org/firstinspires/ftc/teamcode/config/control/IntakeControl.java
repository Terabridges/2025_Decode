package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class IntakeControl implements Control {

    //---------------- Software ----------------
    Intake intake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector toggleRaiser = new EdgeDetector( () -> intake.toggleUseRaiser());

    //---------------- Constructor ----------------
    public IntakeControl(Intake intake, Gamepad gp1, Gamepad gp2){
        this.intake = intake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public IntakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intake, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){

        if(gp1.left_trigger > 0){
            intake.spinnerIn();
        } else if (gp1.right_trigger > 0){
            intake.spinnerOut();
        } else {
            intake.spinnerZero();
        }

        toggleRaiser.update(gp1.a);

    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
