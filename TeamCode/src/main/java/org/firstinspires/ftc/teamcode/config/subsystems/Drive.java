package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor BL;
    public DcMotor BR;
    public DcMotor FL;
    public DcMotor FR;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Drive(HardwareMap map) {
        BL = map.get(DcMotor.class, "BL");
        BR = map.get(DcMotor.class, "BR");
        FL = map.get(DcMotor.class, "FL");
        FR = map.get(DcMotor.class, "FR");
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
