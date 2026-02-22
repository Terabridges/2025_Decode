package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

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
    private double yellowPWM = 0.388;

    //---------------- Constructor ----------------
    public Lights(HardwareMap map) {
        frontLight = map.get(Servo.class, "light1");
        middleLight = map.get(Servo.class, "light2");
        backLight = map.get(Servo.class, "light3");
    }

    //---------------- Methods ----------------
    public void setFrontLight(String color){
        if(color.equals("green")){
            frontLight.setPosition(greenPWM);
        } else if (color.equals("purple")){
            frontLight.setPosition(purplePWM);
        } else if(color.equals("red")){
            frontLight.setPosition(redPWM);
        } else if (color.equals("blue")){
            frontLight.setPosition(bluePWM);
        } else if (color.equals("clear")){
            frontLight.setPosition(clearPWM);
        } else if (color.equals("yellow")){
            frontLight.setPosition(yellowPWM);
        }
    }

    public void setMiddleLight(String color){
        if(color.equals("green")){
            middleLight.setPosition(greenPWM);
        } else if (color.equals("purple")){
            middleLight.setPosition(purplePWM);
        } else if(color.equals("red")){
            middleLight.setPosition(redPWM);
        } else if (color.equals("blue")){
            middleLight.setPosition(bluePWM);
        } else if (color.equals("clear")){
            middleLight.setPosition(clearPWM);
        } else if (color.equals("yellow")){
            middleLight.setPosition(yellowPWM);
        }
    }

    public void setBackLight(String color){
        if(color.equals("green")){
            backLight.setPosition(greenPWM);
        } else if (color.equals("purple")){
            backLight.setPosition(purplePWM);
        } else if(color.equals("red")){
            backLight.setPosition(redPWM);
        } else if (color.equals("blue")){
            backLight.setPosition(bluePWM);
        } else if (color.equals("clear")){
            backLight.setPosition(clearPWM);
        } else if (color.equals("yellow")){
            backLight.setPosition(yellowPWM);
        }
    }

    public void setFrontLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            frontLight.setPosition(bluePWM);
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            frontLight.setPosition(redPWM);
        }
    }

    public void setMiddleLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            middleLight.setPosition(bluePWM);
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            middleLight.setPosition(redPWM);
        }
    }

    public void setBackLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            backLight.setPosition(bluePWM);
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            backLight.setPosition(redPWM);
        }
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        //setFrontLightAlliance();
        //setMiddleLightAlliance();
        //setBackLightAlliance();
    }

    @Override
    public void update(){

    }

}