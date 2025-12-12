package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class TransferControl implements Control {

    //---------------- Software ----------------
    Transfer transfer;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector ballLeft = new EdgeDetector( () -> transfer.ballLeft());
    EdgeDetector ballRight = new EdgeDetector( () -> transfer.ballRight());
    EdgeDetector emptyBalls = new EdgeDetector(()-> transfer.emptyBalls());
    EdgeDetector spindexMode = new EdgeDetector(()-> transfer.toggleSpindexMode());
    EdgeDetector spindexManualLeftRE = new EdgeDetector(()-> transfer.spindexLeft());
    EdgeDetector spindexManualFE = new EdgeDetector(()-> transfer.spindexZero(), true);
    EdgeDetector spindexManualRightRE = new EdgeDetector(()-> transfer.spindexRight());

    //---------------- Constructor ----------------
    public TransferControl(Transfer transfer, Gamepad gp1, Gamepad gp2){
        this.transfer = transfer;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public TransferControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.transfer, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        ballLeft.update(gp1.dpad_left);
        ballRight.update(gp1.dpad_right);
        emptyBalls.update(gp2.x);
        spindexMode.update(gp2.left_stick_button);
        spindexManualLeftRE.update(gp2.dpad_left);
        spindexManualFE.update(gp2.dpad_left);
        spindexManualRightRE.update(gp2.dpad_right);
        spindexManualFE.update(gp2.dpad_right);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Balls", transfer.balls);
        telemetry.addData("Spindex Pos", transfer.spindex.getCurrentPosition());
        telemetry.addData("Spindex Runmode", transfer.spindex.getMode());
        telemetry.addData("ColorDistance", transfer.colorDistance);


        telemetry.addData("Green", transfer.colorSensor.getNormalizedColors().green);
        telemetry.addData("Blue", transfer.colorSensor.getNormalizedColors().blue);
        telemetry.addData("Red", transfer.colorSensor.getNormalizedColors().red);

//        telemetry.addData("Ball Color", transfer.ballColor);
//        telemetry.addData("Sees Red", transfer.isRed);
//        telemetry.addData("color timer seconds", transfer.colorTimer.seconds());
    }
}
