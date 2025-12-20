package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.Util;

public class Transfer implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor spindex;
    public Servo clutch;
    public NormalizedColorSensor colorSensor;
    Util util;

    //---------------- Software ----------------
    public boolean useSpindexPID = true;
    public double spindexManualPow = 0;
    public boolean isClutchDown = false;
    double clutchUp = 0.15;
    double clutchBarelyDown = 0.32; //0.45;
    double clutchDown = 0.55;
    double clutchDownFar = 0.9;
    int ball = 180;

    PIDController spindexController;
    double p = 0.004, i = 0.0000125, d = 0.0003;
    double posTolerance = 5;
    double inteTolerance = 5;
    double spindexPow = 0.0;
    public double spindexTarget = 0.0;
    public double max = 0.4;

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

    public int desiredRotate = 1;
    public boolean isRed = false;

    //---------------- Constructor ----------------
    public Transfer(HardwareMap map) {
        spindex = map.get(DcMotor.class, "spindex");
        clutch = map.get(Servo.class, "clutch");
        colorSensor = map.get(NormalizedColorSensor.class, "color_sensor");
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

    public void toggleSpindexMode(){
        if (spindex.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER){
            spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (spindex.getMode() == DcMotor.RunMode.RUN_USING_ENCODER){
            spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setSpindexPow(double pow){
        spindex.setPower(pow);
    }

    public void spindexRight(){
        useSpindexPID = false;
        spindexManualPow = 0.5;
    }
    public void spindexLeft(){
        useSpindexPID = false;
        spindexManualPow = -0.5;
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
            return (Math.abs(spindexTarget - spindex.getCurrentPosition()) < 7);
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

    public void setClutchBarelyDown(){
        clutch.setPosition(clutchBarelyDown);
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
        if (colorSensor instanceof DistanceSensor) {
            colorDistance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);
        } else {
            colorDistance = Double.POSITIVE_INFINITY;
        }

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

        //previous distance values: 1.92
        if (colorDistance < 1.55 && previousColorDistance > 1.55){
            colorTimer.reset();
            ballDetected = true;
        }

        if (colorDistance > 1.55){
            ballDetected = false;
            ballColor = "none";
        }

        if(ballDetected){
            if (colorTimer.seconds() > 0.03){
                if (green > red && green > blue){
                    ballColor = "green";
                } else {
                    ballColor = "purple";
                }
            }
        }

        if (red > green && red > blue){
            isRed = true;
        } else {
            isRed = false;
        }
    }

    //1 is left, 2 is right, 3 is skip one left then left, 4 is skip one right then right, 5 is skip one left then right
//    public int rotateOrder(){
//        if (motif.equals("PPG")){
//            if(balls.equals("PPG")){
//                return 2;
//            } else if (balls.equals("GPP")){
//                return 1;
//            } else if (balls.equals("PGP")){
//                return 4;
//            }
//        } else if (motif.equals("GPP")){
//            if(balls.equals("PPG")){
//                return 3;
//            } else if (balls.equals("GPP")){
//                return 4;
//            } else if (balls.equals("PGP")){
//                return 2;
//            }
//        } else if (motif.equals("PGP")){
//            if(balls.equals("PPG")){
//                return 1;
//            } else if (balls.equals("GPP")){
//                return 2;
//            } else if (balls.equals("PGP")){
//                return 5;
//            }
//        }
//        return 2;
//    }
    //0 is no rotate, 1 is rotate left, 2 is rotate right
    public int rotateOrder(){
        if (GlobalVariables.motif.equals("PPG")){
            if(balls.equals("PPG")){
                return 1;
            } else if (balls.equals("GPP")){
                return 2;
            } else if (balls.equals("PGP")){
                return 0;
            }
        } else if (GlobalVariables.motif.equals("GPP")){
            if(balls.equals("PPG")){
                return 0;
            } else if (balls.equals("GPP")){
                return 1;
            } else if (balls.equals("PGP")){
                return 2;
            }
        } else if (GlobalVariables.motif.equals("PGP")){
            if(balls.equals("PPG")){
                return 2;
            } else if (balls.equals("GPP")){
                return 0;
            } else if (balls.equals("PGP")){
                return 1;
            }
        }
        return 0;
    }

    public void toggleAutoIntake(){
        autoIntake = !autoIntake;
    }

    public void updateTwoBall(){
        if(balls.equals("PEG")){
            ballList[1] = "P";
        } else if (balls.equals("GEP")){
            ballList[1] = "P";
        } else if (balls.equals("PEP")){
            ballList[1] = "G";
        } else if(balls.equals("EPG")){
            ballList[0] = "P";
        } else if (balls.equals("EGP")){
            ballList[0] = "P";
        } else if (balls.equals("EPP")){
            ballList[0] = "G";
        } else if(balls.equals("PGE")){
            ballList[2] = "P";
        } else if (balls.equals("GPE")){
            ballList[2] = "P";
        } else if (balls.equals("PPE")){
            ballList[2] = "G";
        }
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
        if (isDetecting){
            desiredRotate = rotateOrder();
            updateTwoBall();
        }
    }

}
