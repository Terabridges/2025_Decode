package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Lights implements Subsystem {

    //---------------- Hardware ----------------
    private Servo frontLight;
    private Servo middleLight;
    private Servo backLight;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Lights(HardwareMap map) {
        frontLight = map.get(Servo.class, "light1");
        middleLight = map.get(Servo.class, "light2");
        backLight = map.get(Servo.class, "light3");
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
