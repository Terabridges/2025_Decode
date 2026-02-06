package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Vision implements Subsystem {

    //---------------- Hardware ----------------
    private Limelight3A limelight;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Vision(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");
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
