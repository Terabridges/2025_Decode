package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;

public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo leftTurret;
    private CRServo rightTurret;
    private AnalogInput turretAnalog;
    private AbsoluteAnalogEncoder turretEnc;

    //---------------- Software ----------------
    private double commandedPower = 0.0;
    private double commandedLeftPower = 0.0;
    private double commandedRightPower = 0.0;


    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(CRServo.class, "turretL");
        rightTurret = map.get(CRServo.class, "turretR");
        rightTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
    }

    //---------------- Methods ----------------
    public void moveTurretPow(double pow){
        moveTurretPows(pow, pow);
    }

    public void moveTurretPows(double leftPow, double rightPow){
        commandedLeftPower = leftPow;
        commandedRightPower = rightPow;
        commandedPower = (leftPow + rightPow) / 2.0;
        leftTurret.setPower(leftPow);
        rightTurret.setPower(rightPow);
    }

    public void moveLeftTurretPow(double pow){
        moveTurretPows(pow, commandedRightPower);
    }

    public void moveRightTurretPow(double pow){
        moveTurretPows(commandedLeftPower, pow);
    }

    public double getPositionDeg() {
        return turretEnc.getCurrentPosition();
    }

    public double getAnalogVoltage() {
        return turretEnc.getVoltage();
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public double getCommandedLeftPower() {
        return commandedLeftPower;
    }

    public double getCommandedRightPower() {
        return commandedRightPower;
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
