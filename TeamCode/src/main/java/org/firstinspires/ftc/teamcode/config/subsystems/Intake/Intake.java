package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Intake implements Subsystem {

    //---------------- Hardware ----------------
    public DcMotor megaSpin;
    public Spindex spindex;
    public Spinner spinner;
    public Clutch clutch;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        megaSpin = map.get(DcMotor.class, "intake");
        spindex = new Spindex(map);
        spinner = new Spinner(map);
        clutch = new Clutch(map);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        spindex.toInit();
        clutch.toInit();
        spinner.toInit();
    }

    @Override
    public void update(){
        spindex.update();
        clutch.update();
        spinner.update();
    }

}
