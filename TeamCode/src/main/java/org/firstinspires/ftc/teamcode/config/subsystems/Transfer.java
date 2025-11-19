package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.utility.Util;

public class Transfer implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spindex;
    public Servo clutch;
    public RevColorSensorV3 colorSensor;
    Util util;

    //---------------- Software ----------------
    public boolean useSpindexPID = true;
    double spindexManualPow = 0;
    public boolean isClutchDown = false;
    double clutchUp = 0.42;
    double clutchDown = 0.55;
    int ball = 180;

    PIDController spindexController;
    double p = 0.0017, i = 0.0000125, d = 0.0;
    double posTolerance = 5;
    double inteTolerance = 5;
    double spindexPow = 0.0;
    double spindexTarget = 0.0;
    double max = 0.225;

    NormalizedRGBA colors;
    float red;
    float green;
    float blue;
    double colorDistance;
    public boolean ballDetected = false;
    public String ballColor = "none";

    boolean autoIntake = false;

    //---------------- Constructor ----------------
    public Transfer(HardwareMap map) {
        spindex = map.get(DcMotor.class, "spindex");
        clutch = map.get(Servo.class, "clutch");
        colorSensor = map.get(RevColorSensorV3.class, "color_sensor");
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spindexController = new PIDController(p, i, d);
        spindexController.setIntegrationBounds(-inteTolerance, inteTolerance);
        spindexController.setTolerance(posTolerance);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        util = new Util();

        colors = new NormalizedRGBA();
    }

    //---------------- Methods ----------------

    public void setSpindexPow(double pow){
        spindex.setPower(pow);
    }

    public void spindexRight(){
        useSpindexPID = false;
        spindexManualPow = max;
    }
    public void spindexLeft(){
        useSpindexPID = false;
        spindexManualPow = -max;
    }
    public void spindexZero(){
        useSpindexPID = false;
        spindexManualPow = 0.0;
    }

    public void ballRight(){
        useSpindexPID = true;
        spindexTarget += ball;
    }

    public void ballRight(int num){
        useSpindexPID = true;
        spindexTarget += (num * ball);
    }

    public boolean spindexAtTarget(){
        if(!useSpindexPID){
            return false;
        } else {
            return (Math.abs(spindexTarget - spindex.getCurrentPosition()) < 6);
        }
    }

    public void ballLeft(){
        useSpindexPID = true;
        spindexTarget -= ball;
    }

    public void ballLeft(int num){
        useSpindexPID = true;
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
        spindexPow = util.clamp(spindexPow, -max, max);
        return spindexPow;
    }

    public void setColorValues(){
        colors = colorSensor.getNormalizedColors();
        red = colors.red;
        green = colors.green;
        blue = colors.blue;
        colorDistance = colorSensor.getDistance(DistanceUnit.INCH);

        if (colorDistance < 1.92){
            ballDetected = true;
            if (green > red && green > blue){
                ballColor = "green";
            } else {
                ballColor = "purple";
            }
        } else {
            ballDetected = false;
            ballColor = "none";
        }
    }

    public void toggleAutoIntake(){
        autoIntake = !autoIntake;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setClutchUp();
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update(){

        if (useSpindexPID){
            setSpindexPow(setSpindexPID(spindexTarget));
        } else {
            setSpindexPow(spindexManualPow);
        }
        setColorValues();

        if(autoIntake){
            if (ballDetected && spindexAtTarget()){
                ballLeft();
            }
        }
    }

}
