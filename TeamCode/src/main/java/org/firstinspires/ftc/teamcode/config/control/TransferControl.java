package org.firstinspires.ftc.teamcode.config.control;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class TransferControl implements Control {

    //---------------- Software ----------------
    Transfer transfer;
    GamepadView gp1;
    GamepadView gp2;
    EdgeDetector ballLeft = new EdgeDetector( () -> transfer.ballLeft());
    EdgeDetector ballRight = new EdgeDetector( () -> transfer.ballRight());
    EdgeDetector emptyBalls = new EdgeDetector(()-> transfer.emptyBalls());
    EdgeDetector spindexMode = new EdgeDetector(()-> transfer.toggleSpindexMode());
    EdgeDetector spindexManualLeftRE = new EdgeDetector(()-> transfer.spindexLeft());
    EdgeDetector spindexManualFE = new EdgeDetector(()-> transfer.spindexZero(), true);
    EdgeDetector spindexManualRightRE = new EdgeDetector(()-> transfer.spindexRight());

    //---------------- Constructor ----------------
    public TransferControl(Transfer transfer, GamepadView gp1, GamepadView gp2){
        this.transfer = transfer;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        ballLeft.update(gp1.dpadLeft());
        ballRight.update(gp1.dpadRight());
        emptyBalls.update(gp2.x());
        spindexMode.update(gp2.leftStickButton());
        spindexManualLeftRE.update(gp2.dpadLeft());
        spindexManualFE.update(gp2.dpadLeft());
        spindexManualRightRE.update(gp2.dpadRight());
        spindexManualFE.update(gp2.dpadRight());
    }

    @Override
    public void addTelemetry(TelemetrySink telemetry){
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
