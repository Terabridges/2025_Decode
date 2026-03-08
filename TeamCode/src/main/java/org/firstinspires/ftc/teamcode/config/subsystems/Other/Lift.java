package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;

public class Lift implements Subsystem {

    //---------------- Hardware ----------------
    private Servo kickerLeft;
    private Servo kickerRight;
    private AnalogInput kickerAnalog;
    private AbsoluteAnalogEncoder kickerEnc;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Lift(HardwareMap map) {
        kickerLeft = map.get(Servo.class, "kickL");
        kickerRight = map.get(Servo.class, "kickR");
        kickerAnalog = map.get(AnalogInput.class, "kickAnalog");
        kickerEnc = new AbsoluteAnalogEncoder(kickerAnalog, 3.3, 0, 1);
        kickerLeft.setDirection(Servo.Direction.FORWARD);
        kickerRight.setDirection(Servo.Direction.FORWARD);
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
