package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;

public class Intake implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spinner;
    public CRServo raiserLeft;
    public CRServo raiserRight;
    public AnalogInput raiserAnalog;
    public AbsoluteAnalogEncoder raiserEnc;


    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        spinner = map.get(DcMotor.class, "spinner");
        raiserLeft = map.get(CRServo.class, "raiser_left");
        raiserRight = map.get(CRServo.class, "raiser_right");
        raiserAnalog = map.get(AnalogInput.class, "raiser_analog");
        raiserEnc = new AbsoluteAnalogEncoder(raiserAnalog, 3.3, 0, 1);
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
