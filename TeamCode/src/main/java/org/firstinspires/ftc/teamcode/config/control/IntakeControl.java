package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utility.EdgeDetector;

public class IntakeControl implements Control {

    //---------------- Software ----------------
    Intake intake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector intakeDown = new EdgeDetector( () -> intake.setRaiserDown());
    EdgeDetector intakeDownFE = new EdgeDetector( () -> intake.useRaiserFalse(), true);

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

        intakeDown.update(gp1.left_bumper);
        intakeDownFE.update(gp1.left_bumper);

    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
