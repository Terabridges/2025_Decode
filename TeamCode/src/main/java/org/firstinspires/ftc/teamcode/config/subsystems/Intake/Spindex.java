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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@Configurable
@PsiKitFieldAutoLog
public class Spindex implements Subsystem {

    public static double encoderOffsetDeg = 0.0;
    public static double absoluteEncoderGearRatio = 2.05;
    public static boolean absoluteEncoderInverted = true;
    public static double commandGearRatio = 1.944; //1.75
    public static double commandBiasDeg = -23 ; //-56.0;
    public static boolean invertRight = false;
    public static double rightServoOffset = 0.012;

    public static double turretServoPwmMinUs = 500.0;
    public static double turretServoPwmMaxUs = 2500.0;
    public boolean useSortingSpindex = true;

    public boolean favorFront = true;

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

    private double forwardOne = 175;
    private double forwardTwo = forwardOne + 120;
    private double forwardThree = forwardTwo + 120;

    private double backwardOne = 5; //3
    private double backwardTwo = backwardOne + 120;
    private double backwardThree = backwardTwo + 120;

    private double shootOnePre = 28; //+25
    private double shootOne = shootOnePre + 60;
    private double shootTwoPre = shootOne + 60;
    private double shootTwo = shootTwoPre + 60;
    private double shootThreePre = shootTwo + 60;
    private double shootThree = shootThreePre + 60;

    private double shootOnePreWrap = shootOnePre + 360;
    private double shootOneWrap = shootOne + 360;
    private double shootTwoPreWrap = shootTwoPre + 360;
    private double shootTwoWrap = shootTwo + 360;
    private double shootThreePreWrap = shootThreePre + 360;
    private double shootThreeWrap = shootThree + 360;
    //0 to 620

    //increasing goes clockwise
    //Back is front + 180

    private String currentDirection = "forward";

    private String currentBall = "one";

    private boolean shootMode = false;

    public String currentSpindexServo = "both";

    public String[] ballList = {"G", "P", "P"};
    public String balls = "";

    private double frontGreenThresh = 0.001; //If green is highest, ball is green
    private double frontBlueThresh = 0.001; //If blue is highest, ball is purple
    private double backGreenThresh = 0.001;
    private double backBlueThresh = 0.001;
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
    @PsiKitFieldAutoLog
    private double commandedServoPos = 0.0;
    @PsiKitFieldAutoLog
    private double commandedLeftServoPos = 0.0;
    @PsiKitFieldAutoLog
    private double commandedRightServoPos = 0.0;
    @PsiKitFieldAutoLog
    private double commandedDegree = 0.0;


    //---------------- Constructor ----------------

    public Spindex(HardwareMap map) {
        spindexLeft = map.get(Servo.class, "spindexL");
        spindexRight = map.get(Servo.class, "spindexR");
        applyTurretServoPwmRange(spindexLeft);
        applyTurretServoPwmRange(spindexRight);

        frontColor = map.get(RevColorSensorV3.class, "color1");
        middleColor = map.get(RevColorSensorV3.class, "color2");
        backColor = map.get(RevColorSensorV3.class, "color3");
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, encoderOffsetDeg, absoluteEncoderGearRatio);
        spindexEnc.setInverted(absoluteEncoderInverted);
        spindexLeft.setDirection(Servo.Direction.FORWARD);
        spindexRight.setDirection(Servo.Direction.FORWARD);
        frontColors = new NormalizedRGBA();
        middleColors = new NormalizedRGBA();
        backColors = new NormalizedRGBA();
    }

    //---------------- Methods ----------------
    private void setSpindexPos(double pos){
        commandedServoPos = clampBasePosToSharedRange(pos);

        double leftPos = clamp01(commandedServoPos);
        double rightBasePos = invertRight ? (1.0 - commandedServoPos) : commandedServoPos;
        double rightPos = clamp01(rightBasePos + rightServoOffset);
        commandedLeftServoPos = leftPos;
        commandedRightServoPos = rightPos;

        if (currentSpindexServo.equals("both")) {
            spindexLeft.setPosition(leftPos);
            spindexRight.setPosition(rightPos);
        } else if (currentSpindexServo.equals("left")){
            spindexLeft.setPosition(leftPos);
        } else if (currentSpindexServo.equals("right")){
            spindexRight.setPosition(rightPos);
        }
    }

    public void switchCurrentSpindexServo(){
        if(currentSpindexServo.equals("both")){
            currentSpindexServo = "left";
            ((PwmControl) spindexLeft).setPwmEnable();
            ((PwmControl) spindexRight).setPwmDisable();
        } else if(currentSpindexServo.equals("left")){
            currentSpindexServo = "right";
            ((PwmControl) spindexLeft).setPwmDisable();
            ((PwmControl) spindexRight).setPwmEnable();
        } else if(currentSpindexServo.equals("right")){
            currentSpindexServo = "both";
            ((PwmControl) spindexLeft).setPwmEnable();
            ((PwmControl) spindexRight).setPwmEnable();
        }
    }

    public void setSpindexDegree(double degree){
        double physicalRangeDeg = 360.0 * Math.max(1e-6, Math.abs(absoluteEncoderGearRatio));
        commandedDegree = wrapDegInRange(degree, physicalRangeDeg);
        double ratio = Math.max(1e-6, Math.abs(commandGearRatio));
        setSpindexPos(commandedDegree / (360.0 * ratio));
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

    public void setSpindexShootOnePre(){
        currentBall = "one";
        setSpindexDegree(shootOnePre);
    }

    public void setSpindexShootTwoPre(){
        currentBall = "two";
        setSpindexDegree(shootTwoPre);
    }

    public void setSpindexShootThreePre(){
        currentBall = "three";
        setSpindexDegree(shootThreePre);
    }

    public void setSpindexShootOneWrap(){
        currentBall = "one";
        setSpindexDegree(shootOneWrap);
    }

    public void setSpindexShootOnePreWrap(){
        currentBall = "one";
        setSpindexDegree(shootOnePreWrap);
    }

    public void setSpindexShootTwoWrap(){
        currentBall = "two";
        setSpindexDegree(shootTwoWrap);
    }

    public void setSpindexShootTwoPreWrap(){
        currentBall = "two";
        setSpindexDegree(shootTwoPreWrap);
    }

    public void setSpindexShootThreeWrap(){
        currentBall = "three";
        setSpindexDegree(shootThreeWrap);
    }

    public void setSpindexShootThreePreWrap(){
        currentBall = "three";
        setSpindexDegree(shootThreePreWrap);
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
        double physicalRangeDeg = 360.0 * Math.max(1e-6, Math.abs(absoluteEncoderGearRatio));
        return wrapDegInRange(spindexEnc.getCurrentPosition() + commandBiasDeg, physicalRangeDeg);
    }

    public boolean isSpindexAtPos(){
        double physicalRangeDeg = 360.0 * Math.max(1e-6, Math.abs(absoluteEncoderGearRatio));
        return Math.abs(wrapSignedDegInRange(getCommandedDegree() - getAbsolutePos(), physicalRangeDeg)) <= 8;
    }

    public double getCommandedPos(){
        double ratio = Math.max(1e-6, Math.abs(commandGearRatio));
        return commandedServoPos * 360.0 * ratio;
    }

    public double getCommandedDegree(){
        return commandedDegree;
    }

    private double clampBasePosToSharedRange(double requestedBasePos) {
        double sharedMin = Math.max(0.0, invertRight ? rightServoOffset : -rightServoOffset);
        double sharedMax = Math.min(1.0, invertRight ? 1.0 + rightServoOffset : 1.0 - rightServoOffset);
        return Math.max(sharedMin, Math.min(requestedBasePos, sharedMax));
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(value, 1.0));
    }

    private static double wrapSignedDegInRange(double deg, double rangeDeg) {
        double range = Math.max(1e-6, Math.abs(rangeDeg));
        return ((deg + range * 0.5) % range + range) % range - range * 0.5;
    }

    private static double wrapDegInRange(double deg, double rangeDeg) {
        double range = Math.max(1e-6, Math.abs(rangeDeg));
        return ((deg % range) + range) % range;
    }
    private void applyTurretServoPwmRange(Servo servo) {
        if (!(servo instanceof PwmControl)) {
            return;
        }
        double lo = Math.min(turretServoPwmMinUs, turretServoPwmMaxUs);
        double hi = Math.max(turretServoPwmMinUs, turretServoPwmMaxUs);
        ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(lo, hi));
    }
    public void updateIntookBall(){
        if(isSpindexAtPos()) {
            if (currentDirection.equals("forward")) {
                if (useSortingSpindex) {
                    updateFrontColors();
                }
                if (currentBall.equals("one")) {
                    if(useSortingSpindex && isFrontGreenBall(frontRed, frontGreen, frontBlue)){
                        ballList[0] = "G";
                        ballOneChanged = true;
                    } else if(useSortingSpindex && isFrontPurpleBall(frontRed, frontGreen, frontBlue)){
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
                    if(useSortingSpindex && isFrontGreenBall(frontRed, frontGreen, frontBlue)){
                        ballList[1] = "G";
                        ballTwoChanged = true;
                    } else if(useSortingSpindex && isFrontPurpleBall(frontRed, frontGreen, frontBlue)){
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
                    if(useSortingSpindex && isFrontGreenBall(frontRed, frontGreen, frontBlue)){
                        ballList[2] = "G";
                        ballThreeChanged = true;
                    } else if(useSortingSpindex && isFrontPurpleBall(frontRed, frontGreen, frontBlue)){
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
                if (useSortingSpindex) {
                    updateBackColors();
                }
                if (currentBall.equals("one")) {
                    if(useSortingSpindex && isBackGreenBall(backRed, backGreen, backBlue)){
                        ballList[0] = "G";
                        ballOneChanged = true;
                    } else if(useSortingSpindex && isBackPurpleBall(backRed, backGreen, backBlue)){
                        ballList[0] = "P";
                        ballOneChanged = true;
                    } else {
                        ballList[0] = "B";
                        ballOneChanged = true;
                    }
                    if (ballList[2].equals("E")) {
                        setSpindexBackwardThree();
                    } else if (ballList[1].equals("E")) {
                        setSpindexBackwardTwo();
                    }
                } else if (currentBall.equals("two")) {
                    if(useSortingSpindex && isBackGreenBall(backRed, backGreen, backBlue)){
                        ballList[1] = "G";
                        ballTwoChanged = true;
                    } else if(useSortingSpindex && isBackPurpleBall(backRed, backGreen, backBlue)){
                        ballList[1] = "P";
                        ballTwoChanged = true;
                    } else {
                        ballList[1] = "B";
                        ballTwoChanged = true;
                    }
                    if (ballList[2].equals("E")) {
                        setSpindexBackwardThree();
                    } else if (ballList[0].equals("E")) {
                        setSpindexBackwardOne();
                    }
                } else if (currentBall.equals("three")) {
                    if(useSortingSpindex && isBackGreenBall(backRed, backGreen, backBlue)){
                        ballList[2] = "G";
                        ballThreeChanged = true;
                    } else if(useSortingSpindex && isBackPurpleBall(backRed, backGreen, backBlue)){
                        ballList[2] = "P";
                        ballThreeChanged = true;
                    } else {
                        ballList[2] = "B";
                        ballThreeChanged = true;
                    }
                    if (ballList[1].equals("E")) {
                        setSpindexBackwardTwo();
                    } else if (ballList[0].equals("E")) {
                        setSpindexBackwardOne();
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
        //return frontColorDistance > 1.5 && frontColorDistance < 3.2;
        return frontColorDistance < 3.2;
    }

    public boolean isBackColorDistanceTripped(){
        //return backColorDistance > 1.5 && backColorDistance < 3.2;
        return backColorDistance < 3.2;
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
        if (GlobalVariables.isRedAlliance()) {
            favorFront = true;
            setSpindexForwardOne();
        } else {
            favorFront = false;
            setSpindexBackwardOne();
        }
    }

    @Override
    public void update(){
        balls = ballList[0] + ballList[1] + ballList[2];
    }

    @Override
    public void logPsiKitData() {
        double physicalRangeDeg = 360.0 * Math.max(1e-6, Math.abs(absoluteEncoderGearRatio));
        double absoluteDeg = getAbsolutePos();
        double commandedDeg = getCommandedDegree();
        double errorDeg = wrapSignedDegInRange(commandedDeg - absoluteDeg, physicalRangeDeg);

        Logger.recordOutput("Subsystems/Intake/Spindex/CurrentDirection", currentDirection);
        Logger.recordOutput("Subsystems/Intake/Spindex/CurrentBall", currentBall);
        Logger.recordOutput("Subsystems/Intake/Spindex/ShootMode", shootMode);
        Logger.recordOutput("Subsystems/Intake/Spindex/Balls", balls);
        Logger.recordOutput("Subsystems/Intake/Spindex/Slot1", ballList[0]);
        Logger.recordOutput("Subsystems/Intake/Spindex/Slot2", ballList[1]);
        Logger.recordOutput("Subsystems/Intake/Spindex/Slot3", ballList[2]);
        Logger.recordOutput("Subsystems/Intake/Spindex/LoadedBallCount", loadedBallCount());
        Logger.recordOutput("Subsystems/Intake/Spindex/CommandedServoSharedPos", commandedServoPos);
        Logger.recordOutput("Subsystems/Intake/Spindex/CommandedLeftServoPos", commandedLeftServoPos);
        Logger.recordOutput("Subsystems/Intake/Spindex/CommandedRightServoPos", commandedRightServoPos);
        Logger.recordOutput("Subsystems/Intake/Spindex/ServoSelection", currentSpindexServo);
        Logger.recordOutput("Subsystems/Intake/Spindex/CommandedDegree", commandedDeg);
        Logger.recordOutput("Subsystems/Intake/Spindex/AbsoluteDegree", absoluteDeg);
        Logger.recordOutput("Subsystems/Intake/Spindex/ErrorDegree", errorDeg);
        Logger.recordOutput("Subsystems/Intake/Spindex/AtPosition", isSpindexAtPos());
        Logger.recordOutput("Subsystems/Intake/Spindex/FrontColorDistance", frontColorDistance);
        Logger.recordOutput("Subsystems/Intake/Spindex/BackColorDistance", backColorDistance);
        Logger.recordOutput("Subsystems/Intake/Spindex/FrontRed", frontRed);
        Logger.recordOutput("Subsystems/Intake/Spindex/FrontGreen", frontGreen);
        Logger.recordOutput("Subsystems/Intake/Spindex/FrontBlue", frontBlue);
        Logger.recordOutput("Subsystems/Intake/Spindex/MiddleRed", middleRed);
        Logger.recordOutput("Subsystems/Intake/Spindex/MiddleGreen", middleGreen);
        Logger.recordOutput("Subsystems/Intake/Spindex/MiddleBlue", middleBlue);
        Logger.recordOutput("Subsystems/Intake/Spindex/BackRed", backRed);
        Logger.recordOutput("Subsystems/Intake/Spindex/BackGreen", backGreen);
        Logger.recordOutput("Subsystems/Intake/Spindex/BackBlue", backBlue);
    }
}
