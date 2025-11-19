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
    double hoodDown = 0.0;
    double hoodUp = 1.0;
    public boolean useData = true;
    double maxRPM = 5500;
    private int requiredTagId = -1;

    //---------Targets------
    double turretTarget = 0.0;
    public double targetRPM = 0.0;
    double turretManualPow = 0.0;

    //---------useBool------
    public boolean useTurretLock = false;
    public boolean useTurretPID = false;
    public boolean shooterShoot = false;
    public boolean manualTurret = true;

    //---------PID------
    public PIDController turretLockController;
    double p1 = 0.022, i1 = 0.01, d1 = 0.0;
    double posTolerance1 = 1.2;
    double velTolerance1 = 5.0;
    double inteTolerance1 = 6.0;
    double deadband1 = 0;
    double maxPow1 = 0.135;
    public double turretPower1, error1;

    public PIDController turretController;
    double p2 = 0.0, i2 = 0.0, d2 = 0.0;
    double posTolerance2 = 4;
    double velTolerance2 = 4;
    double inteTolerance2 = 4;
    double deadband2 = 0;
    double maxPow2 = 0.4;
    public double turretPower2, error2;


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

        turretLockController = new PIDController(p1, i1, d1);
        turretLockController.setIntegrationBounds(-inteTolerance1, inteTolerance1);
        turretLockController.setTolerance(posTolerance1, velTolerance1);

        turretController = new PIDController(p2, i2, d2);
        turretController.setIntegrationBounds(-inteTolerance2, inteTolerance2);
        turretController.setTolerance(posTolerance2, velTolerance2);

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

    /** Sets the fiducial ID that the turret lock is allowed to track. Use -1 to accept any tag. */
    public void setRequiredTagId(int tagId) {
        requiredTagId = tagId;
    }

    public double setTurretLockPID(double targetAngle) {
        turretLockController.setPID(p1, i1, d1);
        error1 = vision.getTx();
        if (Math.abs(error1) < deadband1) error1 = 0.0;
        turretPower1 = turretLockController.calculate(error1, targetAngle);
        turretPower1 = util.clamp(turretPower1, -maxPow1, maxPow1);
        return turretPower1;
    }

    public double setTurretPID(double targetAngle) {
        turretController.setPID(p2, i2, d2);
        error2 = 0;
        if (Math.abs(error2) < deadband2) error2 = 0.0;
        turretPower2 = turretController.calculate(error2, targetAngle);
        turretPower2 = util.clamp(turretPower2, -maxPow2, maxPow2);
        return turretPower2;
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

        boolean hasDesiredTarget = vision != null && vision.hasTarget()
                && (requiredTagId < 0 || vision.getCurrentTagId() == requiredTagId);

        if (useTurretLock && hasDesiredTarget) {
            setTurretPower(setTurretLockPID(0.0));
        } else if (useTurretPID){
            setTurretPower(setTurretPID(turretTarget));
        } else if (manualTurret){
            if (!pastPosLimit() && turretManualPow > 0) {
                setTurretPower(turretManualPow);
            } else if (!pastNegLimit() && turretManualPow < 0) {
                setTurretPower(turretManualPow);
            }
        }

        if (useData && vision.hasTarget()){
            double rpmVal =  util.clamp(shooterData.getRPMVal(vision.getDistanceInches()), 0, maxRPM);
            double angleVal = util.clamp(shooterData.getAngleVal(vision.getDistanceInches()), hoodDown, hoodUp);
            if (rpmVal != -2){
                targetRPM = rpmVal;
            }
            if (angleVal != -2) {
                setHoodPos(angleVal);
            }
        }

        if (shooterShoot){
            setShooterRPM(targetRPM);
        } else {
            setShooterRPM(0);
        }
    }
}
