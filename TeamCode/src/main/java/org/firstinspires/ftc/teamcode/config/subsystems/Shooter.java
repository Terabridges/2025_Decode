package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.ShooterData;
import org.firstinspires.ftc.teamcode.config.utility.Util;

public class Shooter implements Subsystem{

    // v = pi*diameter*rpm / 60

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
    public ShooterData shooterData;

    //---------------- Software ----------------!
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;
    double maxPow = 0.18;
    double deadband = 0.15;
    public double turretPower, error;
    double hoodDown = 0.0;
    double hoodUp = 1.0;
    public boolean useData = true;
    double maxRPM = 5500;

    //---------Targets------
    double turretTarget = 0.0;
    public double targetRPM = 0.0;
    double turretManualPow = 0.0;

    //---------useBool------
    public boolean useTurretLock = false;
    public boolean shooterShoot = false;
    public boolean manualTurret = true;

    //---------PID------
    public PIDController turretController;
    double p = 0.035, i = 0.0001, d = 0.0002;
    double posTolerance = 1.2;
    double velTolerance = 5.0;
    double inteTolerance = 6.0;

    //---------------- Constructor ----------------!
    public Shooter(HardwareMap map, Vision vision) {
        turret = map.get(CRServo.class, "turret");
        flyLeft = map.get(DcMotorEx.class, "fly_left");
        flyRight = map.get(DcMotorEx.class, "fly_right");
        hood = map.get(Servo.class, "hood");
        turretAnalog = map.get(AnalogInput.class, "turret_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0.0, 1.0);
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flyRight.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        hood.setDirection(Servo.Direction.REVERSE);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.vision = vision;

        turretController = new PIDController(p, i, d);
        turretController.setIntegrationBounds(-inteTolerance, inteTolerance);
        turretController.setTolerance(posTolerance, velTolerance);
        util = new Util();
        shooterData = new ShooterData();
    }

    //---------------- Methods ----------------!

    //---------Turret------
    public void setTurretPower(double power)
    {
        turret.setPower(power);
    }

    public double getTurretPos()
    {
        return turretEnc.getCurrentPosition();
    }

    public void toggleTurretLock()
    {
        useTurretLock = !useTurretLock;
    }

    public double setTurretPID(double targetAngle) {
        turretController.setPID(p, i, d);
        error = vision.getTx();
        if (Math.abs(error) < deadband) error = 0.0;
        turretPower = turretController.calculate(error, targetAngle);
        turretPower = util.clamp(turretPower, -maxPow, maxPow);
        return turretPower;
    }

    boolean pastPosLimit(){
        return false;
    }

    boolean pastNegLimit(){
        return false;
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

    public void toggleShooter(){
        shooterShoot = !shooterShoot;
    }

    public boolean isAtRPM(){
        return (Math.abs(targetRPM - getShooterRPM()) < 750);
    }

    //---------Hood------
    public void setHoodPos(double pos){
        hood.setPosition(pos);
    }

    public double getHoodPos(){
        return hood.getPosition();
    }

    //---------------- Interface Methods ----------------!
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

        if (useTurretLock) {
            setTurretPower(setTurretPID(0.0));
        } else if (manualTurret){
            if (!pastPosLimit() && turretManualPow > 0) {
                setTurretPower(turretManualPow);
            } else if (!pastNegLimit() && turretManualPow < 0) {
                setTurretPower(turretManualPow);
            } else {
                setTurretPower(turretManualPow);
            }
        }

//        if (useData && vision.hasTarget()){
//            targetRPM = util.clamp(shooterData.getRPMVal(vision.getDistanceInches()), 0, maxRPM);
//            setHoodPos(util.clamp(shooterData.getAngleVal(vision.getDistanceInches()), hoodDown, hoodUp));
//        }

        if (shooterShoot){
            setShooterRPM(targetRPM);
        } else {
            setShooterRPM(0);
        }
    }
}
