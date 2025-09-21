package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;

public class Shooter implements Subsystem{

    //---------------- Hardware ----------------
    public CRServo turret;
    public AnalogInput turretAnalog;
    public AbsoluteAnalogEncoder turretEnc;
    public DcMotor flyLeft;
    public DcMotor flyRight;
    public CRServo hood;
    public AnalogInput hoodAnalog;
    public AbsoluteAnalogEncoder hoodEnc;
    public TouchSensor hoodSwitch;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Shooter(HardwareMap map) {
        turret = map.get(CRServo.class, "turret");
        flyLeft = map.get(DcMotor.class, "fly_left");
        flyRight = map.get(DcMotor.class, "fly_right");
        hood = map.get(CRServo.class, "hood");
        hoodSwitch = map.get(TouchSensor.class, "hood_switch");
        turretAnalog = map.get(AnalogInput.class, "turret_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        hoodAnalog = map.get(AnalogInput.class, "hood_analog");
        hoodEnc = new AbsoluteAnalogEncoder(hoodAnalog, 3.3, 0, 1);
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
