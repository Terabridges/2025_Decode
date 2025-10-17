package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;

public class Intake implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spinner;
    public CRServo raiserLeft;
    public CRServo raiserRight;
    public AnalogInput raiserAnalog;
    public AbsoluteAnalogEncoder raiserEnc;


    //---------------- Software ----------------

    double spinnerTarget = 0.0;
    boolean useSpinner = true;
    double raiserPower = 0.0;
    double raiserTarget = 0.0;
    double currentPos;
    boolean useRaiser = true;

    public PIDController raiserController;
    double p = 0.007, i = 0.001, d = 0.00005;
    double posTolerance = 5.0;
    double inteTolerance = 8.0;

    double raiserUpPos = 190.0;
    double raiserDownPos = 222.0;

    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        spinner = map.get(DcMotor.class, "spinner");
        raiserLeft = map.get(CRServo.class, "intake_left");
        raiserRight = map.get(CRServo.class, "intake_right");
        raiserAnalog = map.get(AnalogInput.class, "intake_analog");
        raiserEnc = new AbsoluteAnalogEncoder(raiserAnalog, 3.3, 0, 1);

        raiserController = new PIDController(p, i, d);
        raiserController.setIntegrationBounds(-inteTolerance, inteTolerance);
        raiserController.setTolerance(posTolerance);
        raiserLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //---------------- Methods ----------------

    public void setSpinnerPower(double pow){
        spinner.setPower(pow);
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

    public double setRaiserPID(double targetPos) {
        raiserController.setPID(p, i, d);
        currentPos = raiserEnc.getCurrentPosition();
        raiserPower = raiserController.calculate(currentPos, targetPos);
        return raiserPower;
    }

    public void setRaiser(double target){
        raiserLeft.setPower(setRaiserPID(target));
        raiserRight.setPower(setRaiserPID(target));
    }

    public void setRaiserUp(){
        useRaiser = true;
        raiserTarget = raiserUpPos;
    }

    public void setRaiserDown(){
        useRaiser = true;
        raiserTarget = raiserDownPos;
    }

    public void zeroRaiser(){
        raiserLeft.setPower(0);
        raiserRight.setPower(0);
    }

    public void useRaiserFalse(){
        useRaiser = false;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setRaiserUp();
    }

    @Override
    public void update(){

        if (useSpinner){
            setSpinnerPower(spinnerTarget);
        }

        if (useRaiser){
            setRaiser(raiserTarget);
        } else {
            zeroRaiser();
        }
    }

}
