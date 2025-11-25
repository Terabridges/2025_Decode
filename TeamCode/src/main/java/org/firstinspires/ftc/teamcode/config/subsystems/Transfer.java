package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double clutchBarelyDown = 0.55;
    double clutchDown = 0.62;
    double clutchDownFar = 0.95;
    int ball = 180;

    PIDController spindexController;
    double p = 0.004, i = 0.0000125, d = 0.0003;
    double posTolerance = 5;
    double inteTolerance = 5;
    double spindexPow = 0.0;
    double spindexTarget = 0.0;
    double max = 0.4;

    NormalizedRGBA colors;
    float red;
    float green;
    float blue;
    public double colorDistance = 0;
    double previousColorDistance = 0;
    public boolean ballDetected = false;
    public String ballColor = "none";

    public boolean autoIntake = true;
    public boolean isDetecting = true;
    public String[] ballList = {"E", "E", "E"};
    public String balls = "";
    public int numBalls = 0;

    public ElapsedTime colorTimer = new ElapsedTime();

    public String motif = "PPG";

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
        colorTimer.reset();
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
        ballRight(1);
        shiftRight();
    }

    public void ballRightSmall(){
        useSpindexPID = true;
        spindexTarget += 80;
    }

    public void ballRight(int num){
        useSpindexPID = true;
        spindexTarget += (num * ball);
    }

    private void shiftRight(){
        String temp = ballList[2];
        ballList[2] = ballList[1];
        ballList[1] = ballList[0];
        ballList[0] = temp;
    }

    private void shiftLeft(){
        String temp = ballList[0];
        ballList[0] = ballList[1];
        ballList[1] = ballList[2];
        ballList[2] = temp;
    }

    public void ballLeft(){
        ballLeft(1);
        shiftLeft();
    }

    public void ballLeft(int num){
        useSpindexPID = true;
        spindexTarget -= (num * ball);
    }

    public void ballLeftSmall(){
        useSpindexPID = true;
        spindexTarget -= 80;
    }

    public boolean spindexAtTarget(){
        if(!useSpindexPID){
            return false;
        } else {
            return (Math.abs(spindexTarget - spindex.getCurrentPosition()) < 6);
        }
    }

    public void setClutchDown(){
        clutch.setPosition(clutchDown);
        isClutchDown = true;
    }

    public void setClutchDownFar(){
        clutch.setPosition(clutchDownFar);
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

    public void emptyBalls(){
        ballList[0] = "E";
        ballList[1] = "E";
        ballList[2] = "E";
        numBalls = 0;
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
        previousColorDistance = colorDistance;
        colorDistance = colorSensor.getDistance(DistanceUnit.INCH);

//        if (colorDistance < 1.92){
//            ballDetected = true;
//            if (green > red && green > blue){
//                ballColor = "green";
//            } else {
//                ballColor = "purple";
//            }
//        } else {
//            ballDetected = false;
//            ballColor = "none";
//        }

        if (colorDistance < 1.92 && previousColorDistance > 1.92){
            colorTimer.reset();
            ballDetected = true;
        }

        if (colorDistance > 1.92){
            ballDetected = false;
            ballColor = "none";
        }

        if(ballDetected){
            if (colorTimer.seconds() > 0.04){
                if (green > red && green > blue){
                    ballColor = "green";
                } else {
                    ballColor = "purple";
                }
            }
        }
    }

    public int rotateOrder(){
        if (motif.equals("PPG")){
            if(balls.equals("PPG")){
                return 0;
            }
        }
        return -1;
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
        if (isDetecting && autoIntake){
            if (numBalls != 3){
                if(ballDetected && spindexAtTarget()){
                    if(ballColor.equals("green")){
                        ballList[0] = "G";
                        numBalls++;
                    } else if (ballColor.equals("purple")){
                        ballList[0] = "P";
                        numBalls++;
                    }

                    if (ballColor.equals("green") || ballColor.equals("purple")) {
                        if (!ballList[1].equals("E") && ballList[2].equals("E")) {
                            ballRight();
                        } else if (ballList[1].equals("E") && !ballList[2].equals("E")) {
                            ballLeft();
                        } else if (ballList[1].equals("E") && ballList[2].equals("E")) {
                            ballRight();
                        }
                    }
                }
            }
        }

        balls = ballList[0] + ballList[1] + ballList[2];
    }

}
