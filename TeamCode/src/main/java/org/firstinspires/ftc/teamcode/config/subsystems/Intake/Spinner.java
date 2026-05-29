package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@Configurable
//@PsiKitFieldAutoLog
public class Spinner implements Subsystem {

    //---------------- Hardware ----------------
    private AnalogInput frontOuterDistanceSensor;
    private AnalogInput frontInnerDistanceSensor;
    private AnalogInput backOuterDistanceSensor;
    private AnalogInput backInnerDistanceSensor;
    private AnalogInput floodgateSensor;
    public DcMotor intakeLeft;

    //---------------- Software ----------------
    private double megaSpinPow = 0;
    private boolean useMegaSpin = true;
    private double frontOuterDistance = 0;
    private double frontInnerDistance = 0;
    private double backOuterDistance = 0;
    private double backInnerDistance = 0;
    public boolean frontOuterTripped = false;
    public boolean frontInnerTripped = false;
    public boolean backOuterTripped = false;
    public boolean backInnerTripped = false;
    public double frontOuterDistanceLowThresh = 0.48;
    public double frontInnerDistanceLowThresh = 0.38;
    public double backOuterDistanceLowThresh = 0.48;
    public double backInnerDistanceLowThresh = 0.38;
    public boolean spinOverride = false;
    private double overridePow = 0;

    public boolean autoSpin = true;
    public static double floodgateMaxCurrentAmps = 80.0;
    //@PsiKitFieldAutoLog
    private double floodgateCurrentAmps = Double.NaN;

    //---------------- Constructor ----------------
    public Spinner(HardwareMap map) {
        frontOuterDistanceSensor = map.get(AnalogInput.class, "distance0");
        frontInnerDistanceSensor = map.get(AnalogInput.class, "distance1");
        backOuterDistanceSensor = map.get(AnalogInput.class, "distance3");
        backInnerDistanceSensor = map.get(AnalogInput.class, "distance2");
        try {
            floodgateSensor = map.get(AnalogInput.class, "floodgate");
        } catch (Exception ignored) {
            floodgateSensor = null;
        }
        intakeLeft = map.get(DcMotor.class, "intakeLeft");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontInnerTimer = new ElapsedTime();
//        backInnerTimer = new ElapsedTime();
    }

    //---------------- Methods ----------------
    private void moveMegaSpinPow(double pow){
        intakeLeft.setPower(pow);
        //intakeRight.setPower(pow);
    }

    public void setMegaSpinPow(double pow){
        megaSpinPow = pow;
    }

    public void setMegaSpinIn(){
        megaSpinPow = 0.98;
        overrideSpinZero();
    }

    public void setMegaSpinOut(){
        megaSpinPow = -0.98;
        overrideSpinZero();
    }

    public void setMegaSpinZero(){
        megaSpinPow = 0;
        overrideSpinZero();
    }

    public void overrideSpinIn(){
        overridePow = 0.8;
        spinOverride = true;
    }

    public void overrideSpinOut(){
        overridePow = -0.8;
        spinOverride = true;
    }

    public void overrideSpinZero(){
        overridePow = 0;
        spinOverride = false;
    }

    public double getFrontOuterDistance(){
        return frontOuterDistance;
    }

    public double getFrontInnerDistance(){
        return frontInnerDistance;
    }

    public double getBackOuterDistance(){
        return backOuterDistance;
    }

    public double getBackInnerDistance(){
        return backInnerDistance;
    }

    public void updateDistances(){
        frontOuterDistance = frontOuterDistanceSensor.getVoltage();
        frontInnerDistance = frontInnerDistanceSensor.getVoltage();
        backOuterDistance = backOuterDistanceSensor.getVoltage();
        backInnerDistance = backInnerDistanceSensor.getVoltage();

        if (frontOuterDistance < frontOuterDistanceLowThresh){
            frontOuterTripped = true;
        }
        if (frontInnerDistance < frontInnerDistanceLowThresh){
            frontInnerTripped = true;
        }
        if (backOuterDistance < backOuterDistanceLowThresh){
            backOuterTripped = true;
        }
        if (backInnerDistance < backInnerDistanceLowThresh){
            backInnerTripped = true;
        }
    }

    private void updateFloodgateCurrent() {
        if (floodgateSensor == null) {
            floodgateCurrentAmps = Double.NaN;
            return;
        }
        floodgateCurrentAmps = (floodgateSensor.getVoltage() / 3.3) * floodgateMaxCurrentAmps;
    }

    public double getFloodgateCurrentAmps() {
        return floodgateCurrentAmps;
    }



    public void unTrip(){
        frontOuterTripped = false;
        frontInnerTripped = false;
        backOuterTripped = false;
        backInnerTripped = false;
    }

    public void toggleAutoSpin(){
        autoSpin = !autoSpin;
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        frontOuterDistance = frontOuterDistanceSensor.getVoltage();
        frontInnerDistance = frontInnerDistanceSensor.getVoltage();
        backOuterDistance = backOuterDistanceSensor.getVoltage();
        backInnerDistance = backInnerDistanceSensor.getVoltage();
        updateFloodgateCurrent();
    }

    @Override
    public void update(){
        if(spinOverride && autoSpin){
            moveMegaSpinPow(overridePow);
        } else if (useMegaSpin){
            moveMegaSpinPow(megaSpinPow);
        }
        updateDistances();
        updateFloodgateCurrent();
    }

    @Override
    public void logPsiKitData() {
        Logger.recordOutput("Subsystems/Intake/Spinner/MegaSpinPower", megaSpinPow);
        Logger.recordOutput("Subsystems/Intake/Spinner/AutoSpin", autoSpin);
        Logger.recordOutput("Subsystems/Intake/Spinner/SpinOverride", spinOverride);
        Logger.recordOutput("Subsystems/Intake/Spinner/OverridePower", overridePow);
        Logger.recordOutput("Subsystems/Intake/Spinner/FrontOuterDistance", frontOuterDistance);
        Logger.recordOutput("Subsystems/Intake/Spinner/FrontInnerDistance", frontInnerDistance);
        Logger.recordOutput("Subsystems/Intake/Spinner/BackOuterDistance", backOuterDistance);
        Logger.recordOutput("Subsystems/Intake/Spinner/BackInnerDistance", backInnerDistance);
        Logger.recordOutput("Subsystems/Intake/Spinner/FrontOuterTripped", frontOuterTripped);
        Logger.recordOutput("Subsystems/Intake/Spinner/FrontInnerTripped", frontInnerTripped);
        Logger.recordOutput("Subsystems/Intake/Spinner/BackOuterTripped", backOuterTripped);
        Logger.recordOutput("Subsystems/Intake/Spinner/BackInnerTripped", backInnerTripped);
        Logger.recordOutput("Subsystems/Intake/Spinner/FloodgateCurrentAmps", floodgateCurrentAmps);
    }

}
