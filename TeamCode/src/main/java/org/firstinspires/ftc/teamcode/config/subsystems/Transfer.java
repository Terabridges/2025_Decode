package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spindex;
    public Servo clutch;
    public RevColorSensorV3 colorSensor;

    //---------------- Software ----------------
    boolean useSpindex = true;
    double spindexPow = 0.0;
    boolean isClutchDown = false;
    double clutchUp = 0.46;
    double clutchDown = 0.525;

    //---------------- Constructor ----------------
    public Transfer(HardwareMap map) {
        spindex = map.get(DcMotor.class, "spindex");
        clutch = map.get(Servo.class, "clutch");
        colorSensor = map.get(RevColorSensorV3.class, "color_sensor");
    }

    //---------------- Methods ----------------

    public void setSpindexPow(double pow){
        spindex.setPower(pow);
    }

    public void spindexRight(){
        spindexPow = 0.3;
    }
    public void spindexLeft(){
        spindexPow = -0.3;
    }
    public void spindexZero(){
        spindexPow = 0.0;
    }

    public void setClutchDown(){
        clutch.setPosition(clutchDown);
        isClutchDown = true;
    }

    public void setClutchUp(){
        clutch.setPosition(clutchUp);
        isClutchDown = false;
    }

    public void toggleClutch(){
        if (isClutchDown){
            setClutchUp();
        } else {
            setClutchDown();
        }
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setClutchUp();
    }

    @Override
    public void update(){

        if (useSpindex){
            setSpindexPow(spindexPow);
        }
    }

}
