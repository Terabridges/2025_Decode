package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;

@Configurable
public class Spindex implements Subsystem {

    //---------------- Hardware ----------------
    private Servo spindexLeft;
    private Servo spindexRight;
    private AnalogInput spindexAnalog;
    private AbsoluteAnalogEncoder spindexEnc;
    private RevColorSensorV3 frontColor;
    private RevColorSensorV3 middleColor;
    private RevColorSensorV3 backColor;
    Util util;

    //---------------- Software ----------------

    private double spindexPos = 0;

    private double spindexDegree = spindexPos*360;

    private double forwardOne = 67;
    private double forwardTwo = 132;
    private double forwardThree = 205;
    private double backwardOne = 170;
    private double backwardTwo = 238;
    private double backwardThree = 100;
    private double forwardZero = 6;
    private double shootOne = 20;
    private double shootTwo = 82;
    private double shootThree = 150;
    private double shootFour = 222;
    private double shootFive = 290;
    private double shootSix = 360;
    //290 //360
    //shoot three half 252


    private String currentDirection = "forward";

    private String currentBall = "one";

    private boolean shootMode = false;


    public String[] ballList = {"G", "P", "P"};
    public String balls = "";

    private double frontGreenThresh = 0.0006; //If green is highest, ball is green was 0.0013 0.0009
    private double frontBlueThresh = 0.0006; //If blue is highest, ball is purple was 0.0013 0.0009
    private double backGreenThresh = 0.0005;
    private double backBlueThresh = 0.0005;
    NormalizedRGBA frontColors;
    public float frontRed = 0;
    public float frontGreen = 0;
    public float frontBlue = 0;
    NormalizedRGBA middleColors;
    public float middleRed = 0;
    public float middleGreen = 0;
    public float middleBlue = 0;
    NormalizedRGBA backColors;
    public float backRed = 0;
    public float backGreen = 0;
    public float backBlue = 0;

    public boolean ballOneChanged = false;
    public boolean ballTwoChanged = false;
    public boolean ballThreeChanged = false;

    private double frontColorDistance = 0;
    private double backColorDistance = 0;


    //---------------- Constructor ----------------

    public Spindex(HardwareMap map) {
        spindexLeft = map.get(Servo.class, "spindexL");
        spindexRight = map.get(Servo.class, "spindexR");
        frontColor = map.get(RevColorSensorV3.class, "color1");
        middleColor = map.get(RevColorSensorV3.class, "color2");
        backColor = map.get(RevColorSensorV3.class, "color3");
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 29, 1.17);
        spindexEnc.setInverted(false);
        spindexRight.setDirection(Servo.Direction.REVERSE);
        frontColors = new NormalizedRGBA();
        middleColors = new NormalizedRGBA();
        backColors = new NormalizedRGBA();
    }

    //---------------- Methods ----------------
    public void setSpindexPos(double pos){
        spindexRight.setPosition(pos);
    }

    public void setSpindexDegree(double degree){
        setSpindexPos(degree/360);
    }


    public void setSpindexForwardOne(){
        currentDirection = "forward";
        currentBall = "one";
        setSpindexDegree(forwardOne);
    }

    public void setSpindexForwardTwo(){
        currentDirection = "forward";
        currentBall = "two";
        setSpindexDegree(forwardTwo);
    }

    public void setSpindexForwardThree(){
        currentDirection = "forward";
        currentBall = "three";
        setSpindexDegree(forwardThree);
    }

    public void setSpindexBackwardOne(){
        currentDirection = "backward";
        currentBall = "one";
        setSpindexDegree(backwardOne);
    }

    public void setSpindexBackwardTwo(){
        currentDirection = "backward";
        currentBall = "two";
        setSpindexDegree(backwardTwo);
    }

    public void setSpindexBackwardThree(){
        currentDirection = "backward";
        currentBall = "three";
        setSpindexDegree(backwardThree);
    }

    public void setSpindexShootOne(){
        currentBall = "one";
        setSpindexDegree(shootOne);
    }

    public void setSpindexShootTwo(){
        currentBall = "two";
        setSpindexDegree(shootTwo);
    }

    public void setSpindexShootThree(){
        currentBall = "three";
        setSpindexDegree(shootThree);
    }

    public void setSpindexShootFour(){
        setSpindexDegree(shootFour);
    }

    public void setSpindexShootFive(){
        setSpindexDegree(shootFive);
    }

    public void setSpindexShootSix(){
        setSpindexDegree(shootSix);
    }

    public void setSpindexForwardZero(){
        setSpindexDegree(forwardZero);
    }

    public void switchSides(){
        if(currentDirection.equals("forward")){
            currentDirection = "backward";
            switch (currentBall) {
                case "one":
                    if(ballList[2].equals("E")) {
                        currentBall = "three";
                        setSpindexBackwardThree();
                    } else if(ballList[1].equals("E")) {
                        currentBall = "two";
                        setSpindexBackwardTwo();
                    } else if(ballList[0].equals("E")) {
                        currentBall = "one";
                        setSpindexBackwardOne();
                    } else {
                        currentBall = "three";
                        setSpindexBackwardThree();
                    }
                    break;
                case "two":
                    if(ballList[0].equals("E")) {
                        currentBall = "one";
                        setSpindexBackwardOne();
                    } else if(ballList[2].equals("E")) {
                        currentBall = "three";
                        setSpindexBackwardThree();
                    } else if(ballList[1].equals("E")) {
                        currentBall = "two";
                        setSpindexBackwardTwo();
                    } else {
                        currentBall = "one";
                        setSpindexBackwardOne();
                    }
                    break;
                case "three":
                    if(ballList[1].equals("E")) {
                        currentBall = "two";
                        setSpindexBackwardTwo();
                    } else if(ballList[2].equals("E")) {
                        currentBall = "three";
                        setSpindexBackwardThree();
                    } else if(ballList[0].equals("E")) {
                        currentBall = "one";
                        setSpindexBackwardOne();
                    } else {
                        currentBall = "two";
                        setSpindexBackwardTwo();
                    }
                    break;
            }
        } else if(currentDirection.equals("backward")){
            currentDirection = "forward";
            switch (currentBall) {
                case "three":
                    if(ballList[0].equals("E")) {
                        currentBall = "one";
                        setSpindexForwardOne();
                    } else if(ballList[1].equals("E")) {
                        currentBall = "two";
                        setSpindexForwardTwo();
                    } else if(ballList[2].equals("E")) {
                        currentBall = "three";
                        setSpindexForwardThree();
                    } else {
                        currentBall = "one";
                        setSpindexForwardOne();
                    }
                    break;
                case "one":
                    if(ballList[1].equals("E")) {
                        currentBall = "two";
                        setSpindexForwardTwo();
                    } else if(ballList[2].equals("E")) {
                        currentBall = "three";
                        setSpindexForwardThree();
                    } else if(ballList[0].equals("E")) {
                        currentBall = "one";
                        setSpindexForwardOne();
                    } else {
                        currentBall = "two";
                        setSpindexForwardTwo();
                    }
                    break;
                case "two":
                    if(ballList[2].equals("E")) {
                        currentBall = "three";
                        setSpindexForwardThree();
                    } else if(ballList[1].equals("E")) {
                        currentBall = "two";
                        setSpindexForwardTwo();
                    } else if(ballList[0].equals("E")) {
                        currentBall = "one";
                        setSpindexForwardOne();
                    } else {
                        currentBall = "three";
                        setSpindexForwardThree();
                    }
                    break;
            }
        }
    }

    public void moveBallClockwise(){

        if (shootMode){
            switch (currentBall) {
                case "one":
                    currentBall = "two";
                    setSpindexShootTwo();
                    break;
                case "two":
                    currentBall = "three";
                    setSpindexShootThree();
                    break;
                case "three":
                    currentBall = "one";
                    setSpindexShootOne();
                    break;
            }
        } else if(currentDirection.equals("forward")){
            switch (currentBall){
                case "one":
                    currentBall = "two";
                    setSpindexForwardTwo();
                    break;
                case "two":
                    currentBall = "three";
                    setSpindexForwardThree();
                    break;
                case "three":
                    currentBall = "one";
                    setSpindexForwardOne();
                    break;
            }
        } else if(currentDirection.equals("backward")) {
            switch (currentBall) {
                case "one":
                    currentBall = "two";
                    setSpindexBackwardTwo();
                    break;
                case "two":
                    currentBall = "three";
                    setSpindexBackwardThree();
                    break;
                case "three":
                    currentBall = "one";
                    setSpindexBackwardOne();
                    break;
            }
        }
    }

    public void moveBallCounter(){
        if (shootMode) {
            switch (currentBall) {
                case "one":
                    currentBall = "three";
                    setSpindexShootThree();
                    break;
                case "two":
                    currentBall = "one";
                    setSpindexShootOne();
                    break;
                case "three":
                    currentBall = "two";
                    setSpindexShootTwo();
                    break;
            }
        } else if(currentDirection.equals("forward")){
            switch (currentBall){
                case "one":
                    currentBall = "three";
                    setSpindexForwardThree();
                    break;
                case "two":
                    currentBall = "one";
                    setSpindexForwardOne();
                    break;
                case "three":
                    currentBall = "two";
                    setSpindexForwardTwo();
                    break;
            }
        } else if(currentDirection.equals("backward")) {
            switch (currentBall) {
                case "one":
                    currentBall = "three";
                    setSpindexBackwardThree();
                    break;
                case "two":
                    currentBall = "one";
                    setSpindexBackwardOne();
                    break;
                case "three":
                    currentBall = "two";
                    setSpindexBackwardTwo();
                    break;
            }
        }
    }

    public void toggleShootMode(){
        if (shootMode){
            shootMode = false;
            if(currentDirection.equals("forward")){
                setSpindexForwardOne();
            } else if (currentDirection.equals("backward")){
                setSpindexBackwardOne();
            }
        } else {
            shootMode = true;
            setSpindexShootOne();
        }
    }

    public String getCurrentDirection(){
        return currentDirection;
    }

    public String getCurrentBall(){
        return currentBall;
    }

    public double getAbsolutePos(){
        return spindexEnc.getCurrentPosition();
    }

    public boolean isSpindexAtPos(){
        return (Math.abs((spindexRight.getPosition()*360) - getAbsolutePos()) <= 7);
    }

    public double getCommandedPos(){
        return spindexRight.getPosition()*360;
    }

    public void updateIntookBall(){
        if(isSpindexAtPos()) {
            if (currentDirection.equals("forward")) {
                updateFrontColors();
                if (currentBall.equals("one")) {
                    if(isFrontGreenBall(frontRed, frontGreen, frontBlue)){
                        ballList[0] = "G";
                        ballOneChanged = true;
                    } else if(isFrontPurpleBall(frontRed, frontGreen, frontBlue)){
                        ballList[0] = "P";
                        ballOneChanged = true;
                    } else {
                        ballList[0] = "B";
                        ballOneChanged = true;
                    }
                    if (ballList[1].equals("E")) {
                        setSpindexForwardTwo();
                    } else if (ballList[2].equals("E")) {
                        setSpindexForwardThree();
                    }
                } else if (currentBall.equals("two")) {
                    if(isFrontGreenBall(frontRed, frontGreen, frontBlue)){
                        ballList[1] = "G";
                        ballTwoChanged = true;
                    } else if(isFrontPurpleBall(frontRed, frontGreen, frontBlue)){
                        ballList[1] = "P";
                        ballTwoChanged = true;
                    } else {
                        ballList[1] = "B";
                        ballTwoChanged = true;
                    }
                    if (ballList[0].equals("E")) {
                        setSpindexForwardOne();
                    } else if (ballList[2].equals("E")) {
                        setSpindexForwardThree();
                    }
                } else if (currentBall.equals("three")) {
                    if(isFrontGreenBall(frontRed, frontGreen, frontBlue)){
                        ballList[2] = "G";
                        ballThreeChanged = true;
                    } else if(isFrontPurpleBall(frontRed, frontGreen, frontBlue)){
                        ballList[2] = "P";
                        ballThreeChanged = true;
                    } else {
                        ballList[2] = "B";
                        ballThreeChanged = true;
                    }
                    if (ballList[0].equals("E")) {
                        setSpindexForwardOne();
                    } else if (ballList[1].equals("E")) {
                        setSpindexForwardTwo();
                    }
                }
            } else if (currentDirection.equals("backward")) {
                updateBackColors();
                if (currentBall.equals("one")) {
                    if(isBackGreenBall(backRed, backGreen, backBlue)){
                        ballList[0] = "G";
                        ballOneChanged = true;
                    } else if(isBackPurpleBall(backRed, backGreen, backBlue)){
                        ballList[0] = "P";
                        ballOneChanged = true;
                    } else {
                        ballList[0] = "B";
                        ballOneChanged = true;
                    }
                    if (ballList[1].equals("E")) {
                        setSpindexBackwardTwo();
                    } else if (ballList[2].equals("E")) {
                        setSpindexBackwardThree();
                    }
                } else if (currentBall.equals("two")) {
                    if(isBackGreenBall(backRed, backGreen, backBlue)){
                        ballList[1] = "G";
                        ballTwoChanged = true;
                    } else if(isBackPurpleBall(backRed, backGreen, backBlue)){
                        ballList[1] = "P";
                        ballTwoChanged = true;
                    } else {
                        ballList[1] = "B";
                        ballTwoChanged = true;
                    }
                    if (ballList[0].equals("E")) {
                        setSpindexBackwardOne();
                    } else if (ballList[2].equals("E")) {
                        setSpindexBackwardThree();
                    }
                } else if (currentBall.equals("three")) {
                    if(isBackGreenBall(backRed, backGreen, backBlue)){
                        ballList[2] = "G";
                        ballThreeChanged = true;
                    } else if(isBackPurpleBall(backRed, backGreen, backBlue)){
                        ballList[2] = "P";
                        ballThreeChanged = true;
                    } else {
                        ballList[2] = "B";
                        ballThreeChanged = true;
                    }
                    if (ballList[0].equals("E")) {
                        setSpindexBackwardOne();
                    } else if (ballList[1].equals("E")) {
                        setSpindexBackwardTwo();
                    }
                }
            }
        }
    }

    public void emptyBalls(){
        ballList[0] = "E";
        ballList[1] = "E";
        ballList[2] = "E";
        ballOneChanged = true;
        ballTwoChanged = true;
        ballThreeChanged = true;
    }

    public void updateFrontColors(){
        frontColors = frontColor.getNormalizedColors();
        frontRed = frontColors.red;
        frontGreen = frontColors.green;
        frontBlue = frontColors.blue;
    }
    public void updateMiddleColors(){
        middleColors = middleColor.getNormalizedColors();
        middleRed = middleColors.red;
        middleGreen = middleColors.green;
        middleBlue = middleColors.blue;
    }
    public void updateBackColors(){
        backColors = backColor.getNormalizedColors();
        backRed = backColors.red;
        backGreen = backColors.green;
        backBlue = backColors.blue;
    }

    public boolean isFrontGreenBall(float red, float green, float blue){
        return (green > frontGreenThresh && green > red && green > blue);
    }

    public boolean isFrontPurpleBall(float red, float green, float blue){
        return (blue > frontBlueThresh && blue > red && blue > green);
    }

    public boolean isBackGreenBall(float red, float green, float blue){
        return (green > backGreenThresh && green > red && green > blue);
    }

    public boolean isBackPurpleBall(float red, float green, float blue){
        return (blue > backBlueThresh && blue > red && blue > green);
    }

    public void updateColorDistances(){
        frontColorDistance = frontColor.getDistance(DistanceUnit.INCH);
        backColorDistance = backColor.getDistance(DistanceUnit.INCH);
    }

    public double getFrontColorDistance(){
        return frontColorDistance;
    }

    public double getBackColorDistance(){
        return backColorDistance;
    }

    public boolean isFrontColorDistanceTripped(){
        return frontColorDistance > 1 && frontColorDistance < 3.5;
    }

    public boolean isBackColorDistanceTripped(){
        return backColorDistance > 1 && backColorDistance < 3.5;
    }

    public int loadedBallCount() {
        int count = 0;
        if (ballList[0] != null && !ballList[0].equals("E")) count++;
        if (ballList[1] != null && !ballList[1].equals("E")) count++;
        if (ballList[2] != null && !ballList[2].equals("E")) count++;
        return count;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setSpindexForwardOne();
    }

    @Override
    public void update(){
        balls = ballList[0] + ballList[1] + ballList[2];
        updateColorDistances();
    }
}
