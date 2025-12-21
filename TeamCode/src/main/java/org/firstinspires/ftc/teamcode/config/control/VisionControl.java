package org.firstinspires.ftc.teamcode.config.control;
import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

public class VisionControl implements Control {

    //---------------- Software ----------------
    Vision vision;
    GamepadView gp1;
    GamepadView gp2;

    //---------------- Constructor ----------------
    public VisionControl(Vision vision, GamepadView gp1, GamepadView gp2){
        this.vision = vision;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){

    }

    @Override
    public void addTelemetry(TelemetrySink telemetry){
        telemetry.addData("Vision Error", vision.getTx());
        telemetry.addData("Distance", vision.getDistanceInches());
        telemetry.addData("Alliance Color", GlobalVariables.allianceColor);
    }
}
