package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Other implements Subsystem {

    //---------------- Hardware ----------------
    private Drive drive;
    private Lift lift;
    private Lights lights;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Other(HardwareMap map) {
        drive = new Drive(map);
        lift = new Lift(map);
        lights = new Lights(map);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        drive.toInit();
        lift.toInit();
        lights.toInit();
    }

    @Override
    public void update(){
        drive.update();
        lift.update();
        lights.update();
    }

}
