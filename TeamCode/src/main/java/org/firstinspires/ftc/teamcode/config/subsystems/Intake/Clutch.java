package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Clutch implements Subsystem {

    //---------------- Hardware ----------------
    private Servo clutch;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Clutch(HardwareMap map) {
        clutch = map.get(Servo.class, "clutch");
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
