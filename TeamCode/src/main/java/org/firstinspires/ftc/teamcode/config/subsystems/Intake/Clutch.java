package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

@Configurable
public class Clutch implements Subsystem {

    //---------------- Hardware ----------------
    private Servo clutch;

    //---------------- Software ----------------
    public static double clutchUp = 0.52;
    public static double clutchDown = 0.4;

    //---------------- Constructor ----------------
    public Clutch(HardwareMap map) {
        clutch = map.get(Servo.class, "clutch");
    }

    //---------------- Methods ----------------
    public void setClutchUp(){
        clutch.setPosition(clutchUp);
    }

    public void setClutchDown(){
        clutch.setPosition(clutchDown);
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setClutchUp();
    }

    @Override
    public void update(){

    }

}
