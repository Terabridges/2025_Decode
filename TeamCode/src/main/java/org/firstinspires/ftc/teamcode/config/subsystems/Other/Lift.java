package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Lift implements Subsystem {

    //---------------- Hardware ----------------
    private DcMotor base;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Lift(HardwareMap map) {
        base = map.get(DcMotor.class, "base");
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
