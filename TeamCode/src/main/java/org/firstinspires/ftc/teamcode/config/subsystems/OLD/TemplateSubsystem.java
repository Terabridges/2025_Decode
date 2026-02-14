package org.firstinspires.ftc.teamcode.config.subsystems.OLD;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TemplateSubsystem implements Subsystem {

    //---------------- Hardware ----------------
    public CRServo servo;

    //---------------- Software ----------------
    public int servoOffset = 0;

    //---------------- Constructor ----------------
    public TemplateSubsystem(HardwareMap map) {
        servo = map.get(CRServo.class, "servo");
    }

    //---------------- Methods ----------------
    public void setServo(double pow){
        servo.setPower(pow);
    }

    public void setServoOn(){
        setServo(1.0);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
