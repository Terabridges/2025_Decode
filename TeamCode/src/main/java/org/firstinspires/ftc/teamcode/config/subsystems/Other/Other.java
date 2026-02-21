package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Lights;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Other implements Subsystem {

    //---------------- Hardware ----------------
    public Drive drive;
    public Lift lift;

    //---------------- Software ----------------

    public boolean unJam = false;

    //---------------- Constructor ----------------
    public Other(HardwareMap map) {
        drive = new Drive(map);
        lift = new Lift(map);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        drive.toInit();
        lift.toInit();
    }

    @Override
    public void update(){
        drive.update();
        lift.update();
    }

}
