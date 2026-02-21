package org.firstinspires.ftc.teamcode.config.control.Intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spinner;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class SpinnerControl implements Control {

    //---------------- Software ----------------
    Spinner spinner;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector SpinInRE = new EdgeDetector( () -> spinner.setMegaSpinIn());
    EdgeDetector SpinInFE = new EdgeDetector( () -> spinner.setMegaSpinZero(), true);
    EdgeDetector SpinOutRE = new EdgeDetector( () -> spinner.setMegaSpinOut());
    EdgeDetector SpinOutFE = new EdgeDetector( () -> spinner.setMegaSpinZero(), true);

    EdgeDetector unTrip = new EdgeDetector( () -> spinner.unTrip());

    //---------------- Constructor ----------------
    public SpinnerControl(Spinner spinner, Gamepad gp1, Gamepad gp2){
        this.spinner = spinner;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public SpinnerControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intake.spinner, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        SpinInRE.update(gp1.left_bumper);
        SpinInFE.update(gp1.left_bumper);
        SpinOutRE.update(gp1.right_bumper);
        SpinOutFE.update(gp1.right_bumper);

        unTrip.update(gp1.left_stick_button);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("FrontOuterTripped", spinner.frontOuterTripped);
        telemetry.addData("FrontInnerTripped", spinner.frontInnerTripped);
        telemetry.addData("BackOuterTripped", spinner.backOuterTripped);
        telemetry.addData("BackInnerTripped", spinner.backInnerTripped);
    }
}
