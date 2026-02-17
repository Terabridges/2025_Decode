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
import org.firstinspires.ftc.teamcode.config.utility.Util;

@Configurable
public class Shooter implements Subsystem {

    //---------------- Hardware ----------------
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private Servo hood;
    private Util util;

    //---------------- Software ----------------
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;

    private double hoodDown = 0.0;
    private double hoodUp = 1.0;
    public static double hoodPos = 0.5;

    public PIDFController flywheelPID;
    private double p = 0.0015, i = 0.0001, d = 0.0, f = 0.0002;
    private double posTolerance = 150;
    private double integrationBounds = 250;
    private double flywheelPower = 0.0;
    public static double flywheelTargetRPM = 2600;
    private double flywheelMaxPower = 1.0;
    public boolean useFlywheelPID = false;
    private double currentRPM = 0;

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
        hood.setPosition(target);
    }

    public double getCurrentRPM(){
        return currentRPM;
    }

    public double getTargetRPM(){
        return flywheelTargetRPM;
    }

    public double getCurrentPower(){
        return flywheelPower;
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
        hood.setPosition(hoodPos);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){
        if (useFlywheelPID){
            setFlywheel(flywheelTargetRPM);
        } else {
            setFlywheelPow(0);
        }
    }

}
