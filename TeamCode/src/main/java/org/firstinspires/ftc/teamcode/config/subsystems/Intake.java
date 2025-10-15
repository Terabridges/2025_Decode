package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;

public class Intake implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spinner;
    public Servo raiserLeft;
    public Servo raiserRight;
//    public AnalogInput raiserAnalog;
//    public AbsoluteAnalogEncoder raiserEnc;


    //---------------- Software ----------------

    double spinnerTarget = 0.0;
    boolean useSpinner = true;

    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        spinner = map.get(DcMotor.class, "spinner");
        raiserLeft = map.get(Servo.class, "intake_left");
        raiserRight = map.get(Servo.class, "intake_right");
//        raiserAnalog = map.get(AnalogInput.class, "raiser_analog");
//        raiserEnc = new AbsoluteAnalogEncoder(raiserAnalog, 3.3, 0, 1);
    }

    //---------------- Methods ----------------

    public void setSpinnerPower(double pow){
        spinnerTarget = pow;
    }

    public void spinnerIn(){
        spinnerTarget = 0.95;
    }

    public void spinnerOut(){
        spinnerTarget = -0.95;
    }

    public void spinnerZero(){
        spinnerTarget = 0.0;
    }



    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

        if (useSpinner){
            setSpinnerPower(spinnerTarget);
        }
    }

}
