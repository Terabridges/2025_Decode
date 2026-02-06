package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Spinner implements Subsystem {

    //---------------- Hardware ----------------
    private AnalogInput frontOuterDistance;
    private AnalogInput frontInnerDistance;
    private AnalogInput backOuterDistance;
    private AnalogInput backInnerDistance;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Spinner(HardwareMap map) {
        frontOuterDistance = map.get(AnalogInput.class, "distance1");
        frontInnerDistance = map.get(AnalogInput.class, "distance2");
        backOuterDistance = map.get(AnalogInput.class, "distance4");
        frontInnerDistance = map.get(AnalogInput.class, "distance3");
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
