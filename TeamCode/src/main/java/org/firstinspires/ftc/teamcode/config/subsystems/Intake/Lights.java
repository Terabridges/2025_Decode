package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

@Configurable
public class Lights implements Subsystem {

    //---------------- Hardware ----------------
    private Servo frontLight;
    private Servo middleLight;
    private Servo backLight;

    //---------------- Software ----------------
    private double greenPWM = 0.475;
    private double purplePWM = 0.720;
    private double redPWM = 0.280;
    private double bluePWM = 0.611;
    private double clearPWM = 0.00;
    public static double flashToggleMs = 750.0;
    private String frontCommandedColor = "clear";
    private String middleCommandedColor = "clear";
    private String backCommandedColor = "clear";
    private boolean flashEnabled = false;
    private boolean flashLightsOn = true;
    private long lastFlashWindow = -1;
    private final ElapsedTime flashTimer = new ElapsedTime();

    //---------------- Constructor ----------------
    public Lights(HardwareMap map) {
        frontLight = map.get(Servo.class, "light1");
        middleLight = map.get(Servo.class, "light2");
        backLight = map.get(Servo.class, "light3");
    }

    //---------------- Methods ----------------
    private double pwmForColor(String color) {
        if ("green".equals(color)) return greenPWM;
        if ("purple".equals(color)) return purplePWM;
        if ("red".equals(color)) return redPWM;
        if ("blue".equals(color)) return bluePWM;
        return clearPWM;
    }

    private void updateFlashWindow() {
        if (!flashEnabled) {
            flashLightsOn = true;
            lastFlashWindow = -1;
            return;
        }
        long window = (long) (flashTimer.milliseconds() / Math.max(1.0, flashToggleMs));
        if (window != lastFlashWindow) {
            lastFlashWindow = window;
            flashLightsOn = (window % 2L) == 0L;
        }
    }

    private void applyFront() {
        frontLight.setPosition((flashEnabled && !flashLightsOn) ? clearPWM : pwmForColor(frontCommandedColor));
    }

    private void applyMiddle() {
        middleLight.setPosition((flashEnabled && !flashLightsOn) ? clearPWM : pwmForColor(middleCommandedColor));
    }

    private void applyBack() {
        backLight.setPosition((flashEnabled && !flashLightsOn) ? clearPWM : pwmForColor(backCommandedColor));
    }

    private void applyAll() {
        applyFront();
        applyMiddle();
        applyBack();
    }

    public void setFlashEnabled(boolean enabled) {
        if (flashEnabled == enabled) return;
        flashEnabled = enabled;
        flashTimer.reset();
        lastFlashWindow = -1;
        updateFlashWindow();
        applyAll();
    }

    public void setFrontLight(String color){
        frontCommandedColor = color;
        applyFront();
    }

    public void setMiddleLight(String color){
        middleCommandedColor = color;
        applyMiddle();
    }

    public void setBackLight(String color){
        backCommandedColor = color;
        applyBack();
    }

    public void setFrontLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            setFrontLight("blue");
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            setFrontLight("red");
        }
    }

    public void setMiddleLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            setMiddleLight("blue");
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            setMiddleLight("red");
        }
    }

    public void setBackLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            setBackLight("blue");
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            setBackLight("red");
        }
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setFlashEnabled(false);
        setFrontLight("clear");
        setMiddleLight("clear");
        setBackLight("clear");
    }

    @Override
    public void update(){
        updateFlashWindow();
        applyAll();
    }

}
