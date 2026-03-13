package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@PsiKitFieldAutoLog
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
        // Helper to map PWM to color name
        private String getColorName(double pwm) {
            if (pwm == greenPWM) return "green";
            if (pwm == purplePWM) return "purple";
            if (pwm == redPWM) return "red";
            if (pwm == bluePWM) return "blue";
            if (pwm == clearPWM) return "clear";
            if (pwm == yellowPWM) return "yellow";
            return "unknown";
        }
    private double clearPWM = 0.00;
    private double yellowPWM = 0.388;
    private double frontCommandedPosition = clearPWM;
    private double middleCommandedPosition = clearPWM;
    private double backCommandedPosition = clearPWM;
            Logger.recordOutput("Subsystems/Intake/Lights/FrontColor", getColorName(frontCommandedPosition));
            Logger.recordOutput("Subsystems/Intake/Lights/MiddleColor", getColorName(middleCommandedPosition));
            Logger.recordOutput("Subsystems/Intake/Lights/BackColor", getColorName(backCommandedPosition));
        frontLight = map.get(Servo.class, "light1");
        middleLight = map.get(Servo.class, "light2");
        backLight = map.get(Servo.class, "light3");
    }

    //---------------- Methods ----------------
    public void setFrontLight(String color){
        if(color.equals("green")){
            frontCommandedPosition = greenPWM;
            frontLight.setPosition(greenPWM);
        } else if (color.equals("purple")){
            frontCommandedPosition = purplePWM;
            frontLight.setPosition(purplePWM);
        } else if(color.equals("red")){
            frontCommandedPosition = redPWM;
            frontLight.setPosition(redPWM);
        } else if (color.equals("blue")){
            frontCommandedPosition = bluePWM;
            frontLight.setPosition(bluePWM);
        } else if (color.equals("clear")){
            frontCommandedPosition = clearPWM;
            frontLight.setPosition(clearPWM);
        } else if (color.equals("yellow")){
            frontCommandedPosition = yellowPWM;
            frontLight.setPosition(yellowPWM);
        }
    }

    public void setMiddleLight(String color){
        if(color.equals("green")){
            middleCommandedPosition = greenPWM;
            middleLight.setPosition(greenPWM);
        } else if (color.equals("purple")){
            middleCommandedPosition = purplePWM;
            middleLight.setPosition(purplePWM);
        } else if(color.equals("red")){
            middleCommandedPosition = redPWM;
            middleLight.setPosition(redPWM);
        } else if (color.equals("blue")){
            middleCommandedPosition = bluePWM;
            middleLight.setPosition(bluePWM);
        } else if (color.equals("clear")){
            middleCommandedPosition = clearPWM;
            middleLight.setPosition(clearPWM);
        } else if (color.equals("yellow")){
            middleCommandedPosition = yellowPWM;
            middleLight.setPosition(yellowPWM);
        }
    }

    public void setBackLight(String color){
        if(color.equals("green")){
            backCommandedPosition = greenPWM;
            backLight.setPosition(greenPWM);
        } else if (color.equals("purple")){
            backCommandedPosition = purplePWM;
            backLight.setPosition(purplePWM);
        } else if(color.equals("red")){
            backCommandedPosition = redPWM;
            backLight.setPosition(redPWM);
        } else if (color.equals("blue")){
            backCommandedPosition = bluePWM;
            backLight.setPosition(bluePWM);
        } else if (color.equals("clear")){
            backCommandedPosition = clearPWM;
            backLight.setPosition(clearPWM);
        } else if (color.equals("yellow")){
            backCommandedPosition = yellowPWM;
            backLight.setPosition(yellowPWM);
        }
    }

    public void setFrontLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            frontCommandedPosition = bluePWM;
            frontLight.setPosition(bluePWM);
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            frontCommandedPosition = redPWM;
            frontLight.setPosition(redPWM);
        }
    }

    public void setMiddleLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            middleCommandedPosition = bluePWM;
            middleLight.setPosition(bluePWM);
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            middleCommandedPosition = redPWM;
            middleLight.setPosition(redPWM);
        }
    }

    public void setBackLightAlliance(){
        if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.BLUE)){
            backCommandedPosition = bluePWM;
            backLight.setPosition(bluePWM);
        } else if (GlobalVariables.getAllianceColor().equals(GlobalVariables.AllianceColor.RED)){
            backCommandedPosition = redPWM;
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

    @Override
    public void logPsiKitData() {
        Logger.recordOutput("Subsystems/Intake/Lights/Alliance", GlobalVariables.getAllianceColorName());
        Logger.recordOutput("Subsystems/Intake/Lights/FrontPosition", frontCommandedPosition);
        Logger.recordOutput("Subsystems/Intake/Lights/MiddlePosition", middleCommandedPosition);
        Logger.recordOutput("Subsystems/Intake/Lights/BackPosition", backCommandedPosition);
    }

}