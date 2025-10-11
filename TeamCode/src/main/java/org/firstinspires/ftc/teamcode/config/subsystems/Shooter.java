package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;

public class Shooter implements Subsystem{

    //TODO merge vision subsystem with shooter subsytem for turret lock, or create seperate subsystem, or figure sum else out
    //---------------- Hardware ----------------
    public CRServo turret;
    public AnalogInput turretAnalog;
    public AbsoluteAnalogEncoder turretEnc;
    public DcMotorEx flyLeft;
    public DcMotorEx flyRight;
    public Servo hood;
    public TouchSensor hoodSwitch;
    public Vision vision;

    //---------------- Software ----------------
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;

    //---------TurretLock-----
    public boolean turretLock = true;


    //---------------- Constructor ----------------
    public Shooter(HardwareMap map) {
        turret = map.get(CRServo.class, "turret");
        flyLeft = map.get(DcMotorEx.class, "fly_left");
        flyRight = map.get(DcMotorEx.class, "fly_right");
        hood = map.get(Servo.class, "hood");
        hoodSwitch = map.get(TouchSensor.class, "hood_switch");
        turretAnalog = map.get(AnalogInput.class, "turret_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        flyRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //---------------- Methods ----------------

    //---------Turret------
    private void setTurretPower(double power)
    {
        turret.setPower(power);
    }

    public double getTurretPos()
    {
        return turretEnc.getCurrentPosition();
    }

    public void toggleTurretLock()
    {
        turretLock = !turretLock;
    }

    //---------Shooter------
    private void setShooterRPM(double RPM){
        double targetVelocity = RPMToVel(RPM);
        flyLeft.setVelocity(targetVelocity);
        flyRight.setVelocity(targetVelocity);
    }

    private void setShooterVel(double tps){
        flyLeft.setVelocity(tps);
        flyRight.setVelocity(tps);
    }

    public double getShooterVel(){
        return flyRight.getVelocity();
    }

    public double getShooterRPM(){
        return velToRPM(flyRight.getVelocity());
    }

    private double velToRPM(double tps){
        double rps = tps / (TICKS_PER_REV * SHOOTER_GEAR_RATIO);
        return rps*60;
    }

    private double RPMToVel(double rpm){
        double tpm = rpm * TICKS_PER_REV * SHOOTER_GEAR_RATIO;
        return tpm/60;
    }

    //---------Hood------
    private void setHoodPos(double pos){
        hood.setPosition(pos);
    }

    public double getHoodPos(){
        return hood.getPosition();
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
