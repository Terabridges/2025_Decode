package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private double forwardOne = 33;
    private double forwardTwo = 101;
    private double forwardThree = 168;
    private double backwardOne = 134;
    private double backwardTwo = 203;
    private double backwardThree = 63;
    private double shootOne = 83;
    private double shootTwo = 151;
    private double shootThree = 218;

    private String currentDirection = "forward";

    private String currentBall = "one";

    private boolean shootMode = false;

    //---------------- Constructor ----------------

    public Spindex(HardwareMap map) {
        spindexLeft = map.get(Servo.class, "spindexL");
        spindexRight = map.get(Servo.class, "spindexR");
        //frontColor = map.get(RevColorSensorV3.class, "color1");
        //middleColor = map.get(RevColorSensorV3.class, "color2");
        //backColor = map.get(RevColorSensorV3.class, "color3");
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 23, 1.17);
        spindexEnc.setInverted(true);
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

    public void switchSides(){
        if(currentDirection.equals("forward")){
            currentDirection = "backward";
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
        } else if(currentDirection.equals("backward")){
            currentDirection = "forward";
            switch (currentBall) {
                case "three":
                    currentBall = "one";
                    setSpindexForwardOne();
                    break;
                case "one":
                    currentBall = "two";
                    setSpindexForwardTwo();
                    break;
                case "two":
                    currentBall = "three";
                    setSpindexForwardThree();
                    break;
            }
        }
    }

    public void moveBallClockwise(){

        if (shootMode){
            switch (currentBall) {
                case "one":
                    currentBall = "three";
                    setSpindexShootThree();
                    break;
                case "three":
                    currentBall = "two";
                    setSpindexShootTwo();
                    break;
                case "two":
                    currentBall = "one";
                    setSpindexShootOne();
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

    public void moveBallCounter(){
        if (shootMode) {
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
        return (Math.abs((spindexRight.getPosition()*360) - getAbsolutePos()) <= 5);
    }

    public double getCommandedPos(){
        return spindexRight.getPosition()*360;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setSpindexForwardOne();
    }

    @Override
    public void update(){

    }
}
