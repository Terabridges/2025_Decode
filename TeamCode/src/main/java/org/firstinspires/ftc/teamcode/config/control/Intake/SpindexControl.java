package org.firstinspires.ftc.teamcode.config.control.Intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class SpindexControl implements Control {

    //---------------- Software ----------------
    Spindex spindex;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector ballClockwise = new EdgeDetector(()-> spindex.moveBallClockwise());
    EdgeDetector ballCounter = new EdgeDetector(()-> spindex.moveBallCounter());
    EdgeDetector switchDirection = new EdgeDetector(()-> spindex.switchSides());
    EdgeDetector toggleShootMode = new EdgeDetector(()-> spindex.toggleShootMode());
    EdgeDetector emptyBalls = new EdgeDetector(()-> spindex.emptyBalls());


    //---------------- Constructor ----------------
    public SpindexControl(Spindex spindex, Gamepad gp1, Gamepad gp2){
        this.spindex = spindex;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public SpindexControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intake.spindex, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        ballClockwise.update(gp1.dpad_right);
        ballCounter.update(gp1.dpad_left);
        switchDirection.update(gp1.dpad_up);
        toggleShootMode.update(gp1.back);
        emptyBalls.update(gp1.right_stick_button);


    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Current Direction", spindex.getCurrentDirection());
        telemetry.addData("Current Ball", spindex.getCurrentBall());
        telemetry.addData("CurrentPos", spindex.getAbsolutePos());
        telemetry.addData("CommandedPos", spindex.getCommandedPos());
        telemetry.addData("Ball Order", spindex.balls);
        telemetry.addData("FrontRed", spindex.frontRed);
        telemetry.addData("FrontGreen", spindex.frontGreen);
        telemetry.addData("FrontBlue", spindex.frontBlue);
        telemetry.addData("BackRed", spindex.backRed);
        telemetry.addData("BackGreen", spindex.backGreen);
        telemetry.addData("BackBlue", spindex.backBlue);
    }
}
