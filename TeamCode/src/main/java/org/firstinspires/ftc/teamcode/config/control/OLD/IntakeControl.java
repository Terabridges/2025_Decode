package org.firstinspires.ftc.teamcode.config.control.OLD;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class IntakeControl implements Control {

    //---------------- Software ----------------
    Intake intake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector IntakeInRE = new EdgeDetector( () -> intake.spinnerIn());
    EdgeDetector IntakeInFE = new EdgeDetector( () -> intake.spinnerZero(), true);
    EdgeDetector IntakeOutRE = new EdgeDetector( () -> intake.spinnerOut());
    EdgeDetector IntakeOutFE = new EdgeDetector( () -> intake.spinnerZero(), true);


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

        IntakeInRE.update(gp1.left_bumper);
        IntakeInFE.update(gp1.left_bumper);
        IntakeOutRE.update(gp1.right_bumper);
        IntakeOutFE.update(gp1.right_bumper);

    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
