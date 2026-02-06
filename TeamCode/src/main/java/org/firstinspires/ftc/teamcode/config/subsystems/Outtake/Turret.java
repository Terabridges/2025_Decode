package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo leftTurret;
    private CRServo rightTurret;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(CRServo.class, "turretL");
        rightTurret = map.get(CRServo.class, "turretR");
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
