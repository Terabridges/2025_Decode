package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@Configurable
//@PsiKitFieldAutoLog
public class Clutch implements Subsystem {

    //---------------- Hardware ----------------
    private Servo clutch;
    public DcMotor clutchSpin;

    //---------------- Software ----------------
    public static double clutchUp = 0.5;
    public static double clutchDown = 0.42;
    public static double clutchDownFar = 0.2;
    private double commandedPosition = clutchUp;
    private boolean isClutchDown = false;
    private boolean isClutchDownFar = false;
    double clutchSpinPow = 0;
    boolean useClutch = true;

    //---------------- Constructor ----------------
    public Clutch(HardwareMap map) {
        clutch = map.get(Servo.class, "clutch");
        clutchSpin = map.get(DcMotor.class, "clutchSpin");
        clutchSpin.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //---------------- Methods ----------------
    public void setClutchUp(){
        commandedPosition = clutchUp;
        clutch.setPosition(clutchUp);
        isClutchDown = false;
        isClutchDownFar = false;
    }

    public void setClutchDown(){
        commandedPosition = clutchDown;
        clutch.setPosition(clutchDown);
        isClutchDown = true;
        isClutchDownFar = false;
    }

    public void setClutchDownFar(){
        commandedPosition = clutchDownFar;
        clutch.setPosition(clutchDownFar);
        isClutchDown = false;
        isClutchDownFar = true;
    }

    public void toggleClutch(){
        if (isClutchDown){
            setClutchUp();
            isClutchDown = false;
        } else {
            setClutchDown();
            isClutchDown = true;
        }
    }

    public void toggleClutchUpFar(){
        if(!isClutchDownFar){
            setClutchDownFar();
            isClutchDownFar = true;
        } else {
            setClutchUp();
            isClutchDownFar = false;
        }
    }

    public void spinClutch(double pow){
        clutchSpin.setPower(pow);
    }

    public void spinClutchIn(){
        clutchSpinPow = 0.98;
    }

    public void spinClutchOut(){
        clutchSpinPow = -0.5;
    }

    public void spinClutchStop(){
        clutchSpinPow = 0;
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        setClutchUp();
    }

    @Override
    public void update(){
        if (useClutch){
            spinClutch(clutchSpinPow);
        }
    }

    @Override
    public void logPsiKitData() {
        Logger.recordOutput("Subsystems/Intake/Clutch/Position", commandedPosition);
        Logger.recordOutput("Subsystems/Intake/Clutch/IsDown", isClutchDown);
        Logger.recordOutput("Subsystems/Intake/Clutch/IsDownFar", isClutchDownFar);
    }

}
