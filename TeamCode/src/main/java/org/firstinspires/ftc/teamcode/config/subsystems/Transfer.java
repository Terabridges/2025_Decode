package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spindex;
    public Servo clutch;
    public RevColorSensorV3 colorSensor;

    //---------------- Software ----------------
    boolean useSpindexPID = true;
    double spindexManualPow = 0;
    boolean isClutchDown = false;
    double clutchUp = 0.46;
    double clutchDown = 0.525;
    int ball = 175;

    PIDController spindexController;
    double p = 0.0012, i = 0, d = 0;
    double posTolerance = 10;
    double inteTolerance = 10;
    double spindexPow = 0.0;
    double spindexTarget = 0.0;
    double max = 0.25;

    //---------------- Constructor ----------------
    public Transfer(HardwareMap map) {
        spindex = map.get(DcMotor.class, "spindex");
        clutch = map.get(Servo.class, "clutch");
        colorSensor = map.get(RevColorSensorV3.class, "color_sensor");
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spindexController = new PIDController(p, i, d);
        spindexController.setIntegrationBounds(-inteTolerance, inteTolerance);
        spindexController.setTolerance(posTolerance);
    }

    //---------------- Methods ----------------

    public void setSpindexPow(double pow){
        spindex.setPower(pow);
    }

    public void spindexRight(){
        spindexManualPow = 0.3;
    }
    public void spindexLeft(){
        spindexManualPow = -0.3;
    }
    public void spindexZero(){
        spindexManualPow = 0.0;
    }

    public void ballRight(){
        spindexTarget += ball;
    }

    public void ballRight(int num){
        spindexTarget += (num * ball);
    }

    public void ballLeft(){
        spindexTarget -= ball;
    }

    public void ballLeft(int num){
        spindexTarget -= (num * ball);
    }

    public void setClutchDown(){
        clutch.setPosition(clutchDown);
        isClutchDown = true;
    }

    public void setClutchUp(){
        clutch.setPosition(clutchUp);
        isClutchDown = false;
    }

    public void toggleClutch(){
        if (isClutchDown){
            setClutchUp();
        } else {
            setClutchDown();
        }
    }

    public double setSpindexPID(double targetPos) {
        spindexController.setPID(p, i, d);
        double currentPos = spindex.getCurrentPosition();
        spindexPow = spindexController.calculate(currentPos, targetPos);
        spindexPow = clamp(spindexPow, -max, max);
        return spindexPow;
    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setClutchUp();
    }

    @Override
    public void update(){

        if (useSpindexPID){
            setSpindexPow(setSpindexPID(spindexTarget));
        } else {
            setSpindexPow(spindexManualPow);
        }
    }

}
