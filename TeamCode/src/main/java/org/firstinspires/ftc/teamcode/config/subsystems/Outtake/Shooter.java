package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.ShooterData;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@Configurable
@PsiKitFieldAutoLog
public class Shooter implements Subsystem {

    //---------------- Hardware ----------------
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private Servo hood;
    private Util util;

    //---------------- Software ----------------
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;

    //Hood range 0.43 to 1.0
    private double hoodDown = 0.43;
    private double hoodUp = 1.0;
    public double hoodPos = 0.5;
    private double commandedHoodPos = hoodPos;

    public PIDFController flywheelPID;
    private double p = 0.0015, i = 0.0001, d = 0.0, f = 0.0002;
    private double posTolerance = 150;
    private double integrationBounds = 250;
    private double flywheelPower = 0.0;
    public double flywheelTargetRPM = 2600;
    private double flywheelMaxPower = 1.0;
    public boolean useFlywheelPID = true;
    private double currentRPM = 0;
    public boolean flywheelOverride = false;
    public double flywheelOverrideRPM = 0;
    private double closeRPM = 2300;
    private double closeAngle = 0.7;
    private double farRPM = 3000;
    private double farAngle = 0.91;

    public boolean autoHood = true;

    public double flywheelOffset = 0;
    public double hoodOffset = 0;

    public double newRPM = flywheelTargetRPM + flywheelOffset;

    //---------------- Constructor ----------------
    public Shooter(HardwareMap map) {
        leftFlywheel = map.get(DcMotorEx.class, "fly_left");
        rightFlywheel = map.get(DcMotorEx.class, "fly_right");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = map.get(Servo.class, "hood");

        flywheelPID = new PIDFController(p, i, d, f);
        flywheelPID.setIntegrationBounds(-integrationBounds, integrationBounds);
        flywheelPID.setTolerance(posTolerance);
        util = new Util();
    }

    //---------------- Methods ----------------

    public double velToRPM(double tps) {
        double rps = tps / (TICKS_PER_REV * SHOOTER_GEAR_RATIO);
        return rps * 60;
    }

    public double RPMToVel(double rpm) {
        double tpm = rpm * TICKS_PER_REV * SHOOTER_GEAR_RATIO;
        return tpm / 60;
    }

    public void setFlywheelPow(double pow){
        leftFlywheel.setPower(pow);
        rightFlywheel.setPower(pow);
    }

    public double setFlywheelPID(double targetRPM) {
        flywheelPID.setPIDF(p, i, d, f);
        currentRPM = velToRPM(leftFlywheel.getVelocity());
        flywheelPower = flywheelPID.calculate(currentRPM, targetRPM);
        flywheelPower = util.clamp(flywheelPower, -flywheelMaxPower, flywheelMaxPower);
        return flywheelPower;
    }

    public void setFlywheel(double target){
        setFlywheelPow(setFlywheelPID(target));
    }

    public void setHood(double target){
        commandedHoodPos = target;
        hood.setPosition(target);
    }

    public double getCurrentRPM(){
        return currentRPM;
    }

    public double getTargetRPM(){
        return flywheelTargetRPM + flywheelOffset;
    }

    public double getCurrentPower(){
        return flywheelPower;
    }

    public double getCurrentHoodPosition(){
        return commandedHoodPos;
    }

    public void setLeftFlywheelPow(double pow){
        leftFlywheel.setPower(pow);
    }

    public void setRightFlywheelPow(double pow){
        rightFlywheel.setPower(pow);
    }

    public void toggleUseFlywheel(){
        useFlywheelPID = !useFlywheelPID;
    }

    public void setHoodTarget(){
        setHood(util.clamp(hoodPos+hoodOffset, 0.46, hoodUp));
    }

    public boolean isAtRPM(){
        return (Math.abs(getTargetRPM() - currentRPM) < 200);
    }

    public void toggleCloseOverride(){
        if(flywheelOverride){
            flywheelOverride = false;
            autoHood = true;
        } else {
            flywheelOverride = true;
            autoHood = false;
            flywheelTargetRPM = closeRPM;
            hood.setPosition(closeAngle);
        }
    }

    public void toggleFarOverride(){
        if(flywheelOverride){
            flywheelOverride = false;
            autoHood = true;
        } else {
            flywheelOverride = true;
            autoHood = false;
            flywheelTargetRPM = farRPM;
            hood.setPosition(farAngle);
        }
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){
        newRPM = flywheelTargetRPM + flywheelOffset;
        if (useFlywheelPID){
            if(!flywheelOverride) {
                setFlywheel(newRPM);
            } else {
                setFlywheel(flywheelOverrideRPM+flywheelOffset);
            }
        } else {
            setFlywheelPow(0);
        }
        if (autoHood) {
            setHoodTarget();
        }
    }

    @Override
    public void logPsiKitData() {
        Logger.recordOutput("Subsystems/Outtake/Shooter/UseFlywheelPID", useFlywheelPID);
        Logger.recordOutput("Subsystems/Outtake/Shooter/FlywheelOverride", flywheelOverride);
        Logger.recordOutput("Subsystems/Outtake/Shooter/FlywheelTargetRPM", newRPM);
        Logger.recordOutput("Subsystems/Outtake/Shooter/FlywheelCurrentRPM", currentRPM);
        Logger.recordOutput("Subsystems/Outtake/Shooter/FlywheelPower", flywheelPower);
        Logger.recordOutput("Subsystems/Outtake/Shooter/AtRPM", isAtRPM());
        Logger.recordOutput("Subsystems/Outtake/Shooter/AutoHood", autoHood);
        Logger.recordOutput("Subsystems/Outtake/Shooter/HoodTarget", hoodPos);
        Logger.recordOutput("Subsystems/Outtake/Shooter/HoodServoPosition", commandedHoodPos);
    }

}
