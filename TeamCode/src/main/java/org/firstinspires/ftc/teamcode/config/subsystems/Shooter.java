package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.utility.Util;

public class Shooter implements Subsystem{

    //---------------- Hardware ----------------!
    public CRServo turret;
    public AnalogInput turretAnalog;
    public AbsoluteAnalogEncoder turretEnc;
    public DcMotorEx flyLeft;
    public DcMotorEx flyRight;
    public Servo hood;
    //public TouchSensor hoodSwitch;

    //---------Objects------
    public Vision vision;
    public Util util;

    //---------------- Software ----------------!
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;
    double maxPow = 0.6;
    double deadband = 0.18;
    public double turretPower, error;

    //---------Targets------
    double turretTarget = 0.0;
    double targetRPM = 0.0;
    double hoodTarget = 0.0;

    //---------useBool------
    boolean useTurretPID = true;
    public boolean useTurretLock = false;
    boolean shooterShoot = true;

    //---------PID------
    public PIDController turretController;
    double p = 0.02, i = 0.00015, d = 0.0009;
    double posTolerance = 1.2;
    double velTolerance = 5.0;
    double inteTolerance = 6.0;

    //---------------- Constructor ----------------!
    public Shooter(HardwareMap map, Vision vision) {
        turret = map.get(CRServo.class, "turret");
        flyLeft = map.get(DcMotorEx.class, "fly_left");
        flyRight = map.get(DcMotorEx.class, "fly_right");
        hood = map.get(Servo.class, "hood");
        //hoodAnalog: hood_analog
        //hoodSwitch = map.get(TouchSensor.class, "hood_switch");
        turretAnalog = map.get(AnalogInput.class, "turret_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        flyRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.vision = vision;

        turretController = new PIDController(p, i, d);
        turretController.setIntegrationBounds(-inteTolerance, inteTolerance);
        turretController.setTolerance(posTolerance, velTolerance);
    }

    //---------------- Methods ----------------!

    //---------Turret------
    public void setTurretPower(double power)
    {
        turret.setPower(power);
    }

    public void setTurret(double target){
        turret.setPower(setTurretPID(target));
    }

    public double getTurretPos()
    {
        return turretEnc.getCurrentPosition();
    }

//    public double getTurretHeading()
//    {
//        double turretHeading = 67; //TODO return actual turret heading
//        return turretHeading;
//    }

    public void toggleTurretLock()
    {
        useTurretLock = !useTurretLock;
    }

    public double setTurretPID(double targetAngle) {
        turretController.setPID(p, i, d);
        error = vision.getTx();
        if (Math.abs(error) < deadband) error = 0.0;
        turretPower = turretController.calculate(error, targetAngle);
        turretPower = clamp(turretPower, -maxPow, maxPow);
        return turretPower;
    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    //---------Shooter------
    public void setShooterRPM(double RPM){
        double targetVelocity = RPMToVel(RPM);
        flyLeft.setVelocity(targetVelocity);
        flyRight.setVelocity(targetVelocity);
    }

    public void setShooterVel(double tps){
        flyLeft.setVelocity(tps);
        flyRight.setVelocity(tps);
    }

    public double getShooterVel(){
        return flyRight.getVelocity();
    }

    public double getShooterRPM(){
        return velToRPM(flyRight.getVelocity());
    }

    public double velToRPM(double tps){
        double rps = tps / (TICKS_PER_REV * SHOOTER_GEAR_RATIO);
        return rps*60;
    }

    public double RPMToVel(double rpm){
        double tpm = rpm * TICKS_PER_REV * SHOOTER_GEAR_RATIO;
        return tpm/60;
    }

    public void bumpShooterUp(){
        targetRPM += 1000;
        if (targetRPM == 6000){
            targetRPM = 0;
        }
    }

    public void bumpShooterDown(){
        targetRPM -= 1000;
        if (targetRPM == -1000){
            targetRPM = 5000;
        }
    }

    //---------Hood------
    public void setHoodPos(double pos){
        hood.setPosition(pos);
    }

    public double getHoodPos(){
        return hood.getPosition();
    }

//    public boolean isHoodSensorOn(){
//        return hoodSwitch.isPressed();
//    }


    //---------------- Interface Methods ----------------!
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

        if (useTurretLock){
            turretTarget = 0.0;
            turret.setPower(setTurretPID(turretTarget));
        }

        if (shooterShoot){
            setShooterRPM(targetRPM);
        }
    }
}
