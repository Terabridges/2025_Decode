package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;

public class Intake implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spinner;
    public CRServo raiserRight;
    public AnalogInput raiserAnalog;
    public AbsoluteAnalogEncoder raiserEnc;
    Util util;


    //---------------- Software ----------------

    double spinnerTarget = 0.0;
    boolean useSpinner = true;
    double raiserPower = 0.0;
    double raiserTarget = 0.0;
    double currentPos;
    boolean useRaiser = true;
    public boolean spinnerMacro = false;
    public double spinnerMacroTarget = 0;

    public PIDController raiserController;
    double p = 0.002, i = 0.0, d = 0.0; //p = 0.003
    double posTolerance = 3.0;
    double inteTolerance = 8.0;
    double raiserMaxPow = 0.15;

    double raiserUpPos = 210;
    double raiserDownPos = 175;

    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        spinner = map.get(DcMotor.class, "spinner");
        raiserRight = map.get(CRServo.class, "intake_right");
        raiserAnalog = map.get(AnalogInput.class, "intake_analog");
        raiserRight.setDirection(DcMotorSimple.Direction.REVERSE);
        raiserEnc = new AbsoluteAnalogEncoder(raiserAnalog, 3.3, 0, 1);

        raiserController = new PIDController(p, i, d);
        raiserController.setIntegrationBounds(-inteTolerance, inteTolerance);
        raiserController.setTolerance(posTolerance);
        util = new Util();
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
        raiserPower = util.clamp(raiserPower, -raiserMaxPow, raiserMaxPow);
        return raiserPower;
    }

    public void setRaiser(double target){
        raiserRight.setPower(setRaiserPID(target));
    }

    public void zeroRaiser(){
        raiserRight.setPower(0);
    }

    public void toggleUseRaiser(){
        useRaiser = !useRaiser;
    }

    public void holdRaiserDown(){
        raiserTarget = raiserDownPos;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        holdRaiserDown();
    }

    @Override
    public void update(){

        if (spinnerMacro){
            setSpinnerPower(spinnerMacroTarget);
        } else if (useSpinner){
            setSpinnerPower(spinnerTarget);
        }

        if (useRaiser){
            setRaiser(raiserTarget);
        } else {
            zeroRaiser();
        }
    }

}
