package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

@Configurable
public class Spinner implements Subsystem {

    //---------------- Hardware ----------------
    private AnalogInput frontOuterDistanceSensor;
    private AnalogInput frontInnerDistanceSensor;
    private AnalogInput backOuterDistanceSensor;
    private AnalogInput backInnerDistanceSensor;
    public DcMotor megaSpin;

    //---------------- Software ----------------
    private double megaSpinPow = 0;
    private boolean useMegaSpin = true;
    private double frontOuterDistance = 0;
    private double frontInnerDistance = 0;
    private double backOuterDistance = 0;
    private double backInnerDistance = 0;
    private double previousFrontOuterDistance = 0;
    private double previousFrontInnerDistance = 0;
    private double previousBackOuterDistance = 0;
    private double previousBackInnerDistance = 0;
    public boolean frontOuterTripped = false;
    public boolean frontInnerTripped = false;
    public boolean backOuterTripped = false;
    public boolean backInnerTripped = false;
    public static double frontOuterDistanceLowThresh = 0.7; // lower than 0.7
    //public static double frontOuterDistanceHighThresh = 0.35; // higher than 0.32
    public static double frontInnerDistanceLowThresh = 0.3; //lower than 0.3
    public static double backOuterDistanceLowThresh = 0.7; //lower than 0.7
    //public static double backOuterDistanceHighThresh = 0.35; //higher than 0.375
    public static double backInnerDistanceLowThresh = 0.3; //lower than 0.3
    public boolean spinOverride = false;
    private double overridePow = 0;



    //---------------- Constructor ----------------
    public Spinner(HardwareMap map) {
        frontOuterDistanceSensor = map.get(AnalogInput.class, "distance0");
        frontInnerDistanceSensor = map.get(AnalogInput.class, "distance1");
        backOuterDistanceSensor = map.get(AnalogInput.class, "distance3");
        backInnerDistanceSensor = map.get(AnalogInput.class, "distance2");
        megaSpin = map.get(DcMotor.class, "intake");
        megaSpin.setDirection(DcMotorSimple.Direction.REVERSE);
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
        overrideSpinZero();
    }

    public void setMegaSpinOut(){
        megaSpinPow = -0.95;
        overrideSpinZero();
    }

    public void setMegaSpinZero(){
        megaSpinPow = 0;
        overrideSpinZero();
    }

    public void overrideSpinIn(){
        overridePow = 0.95;
        spinOverride = true;
    }

    public void overrideSpinZero(){
        overridePow = 0;
        spinOverride = false;
    }

    public double getFrontOuterDistance(){
        return frontOuterDistance;
    }

    public double getFrontInnerDistance(){
        return frontInnerDistance;
    }

    public double getBackOuterDistance(){
        return backOuterDistance;
    }

    public double getBackInnerDistance(){
        return backInnerDistance;
    }

    public void updateDistances(){
        previousFrontOuterDistance = frontOuterDistance;
        previousFrontInnerDistance = frontInnerDistance;
        previousBackOuterDistance = backOuterDistance;
        previousBackInnerDistance = backInnerDistance;

        frontOuterDistance = frontOuterDistanceSensor.getVoltage();
        frontInnerDistance = frontInnerDistanceSensor.getVoltage();
        backOuterDistance = backOuterDistanceSensor.getVoltage();
        backInnerDistance = backInnerDistanceSensor.getVoltage();

        if (frontOuterDistance < frontOuterDistanceLowThresh){
            frontOuterTripped = true;
        }
        if (frontInnerDistance < frontInnerDistanceLowThresh){
            frontInnerTripped = true;
        }
        if (backOuterDistance < backOuterDistanceLowThresh){
            backOuterTripped = true;
        }
        if (backInnerDistance < backInnerDistanceLowThresh){
            backInnerTripped = true;
        }
    }



    public void unTrip(){
        frontOuterTripped = false;
        frontInnerTripped = false;
        backOuterTripped = false;
        backInnerTripped = false;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        frontOuterDistance = frontOuterDistanceSensor.getVoltage();
        frontInnerDistance = frontInnerDistanceSensor.getVoltage();
        backOuterDistance = backOuterDistanceSensor.getVoltage();
        backInnerDistance = backInnerDistanceSensor.getVoltage();
    }

    @Override
    public void update(){
        if(spinOverride){
            moveMegaSpinPow(overridePow);
        } else if (useMegaSpin){
            moveMegaSpinPow(megaSpinPow);
        }
        updateDistances();
    }

}
