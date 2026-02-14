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
    public DcMotor megaSpin;

    //---------------- Software ----------------
    private double megaSpinPow = 0;
    private boolean useMegaSpin = true;

    //---------------- Constructor ----------------
    public Spinner(HardwareMap map) {
        frontOuterDistance = map.get(AnalogInput.class, "distance0");
        frontInnerDistance = map.get(AnalogInput.class, "distance1");
        backOuterDistance = map.get(AnalogInput.class, "distance2");
        backInnerDistance = map.get(AnalogInput.class, "distance3");
        megaSpin = map.get(DcMotor.class, "intake");
    }

    //---------------- Methods ----------------
    private void moveMegaSpinPow(double pow){
        megaSpin.setPower(pow);
    }

    public void setMegaSpinPow(double pow){
        megaSpinPow = pow;
    }

    public void setMegaSpinIn(){
        megaSpinPow = 0.95;
    }

    public void setMegaSpinOut(){
        megaSpinPow = -0.95;
    }

    public void setMegaSpinZero(){
        megaSpinPow = 0;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){
        if (useMegaSpin){
            moveMegaSpinPow(megaSpinPow);
        }
    }

}
