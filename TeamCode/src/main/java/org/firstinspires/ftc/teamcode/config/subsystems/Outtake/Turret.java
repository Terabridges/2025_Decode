package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;

@Configurable
public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo leftTurret;
    private CRServo rightTurret;
    private AnalogInput turretAnalog;
    private AbsoluteAnalogEncoder turretEnc;
    private Util util;

    //---------------- Software ----------------
    public PIDController turretPID;
    public static double p = 0.0, i = 0.0, d = 0.0;
    public static double posTolerance = 5;
    public static double integrationBounds = 5;
    private double turretPower = 0.0;
    public static double turretTarget = 0.0;
    public static double turretMaxPower = 1.0;
    public static double currentPos = 0;
    private boolean useTurretPID = true;

    private double turretCounterClockwiseLimit = 170;
    private double turretClowckwiseLimit = 60;

    private double TurretForward = 210;

    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(CRServo.class, "turretL");
        rightTurret = map.get(CRServo.class, "turretR");
        rightTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);

        turretPID = new PIDController(p, i, d);
        turretPID.setIntegrationBounds(-integrationBounds, integrationBounds);
        turretPID.setTolerance(posTolerance);
        util = new Util();
    }

    //---------------- Methods ----------------
    public void setTurretPow(double pow){
        leftTurret.setPower(pow);
        rightTurret.setPower(pow);
    }

    public double setTurretPID(double targetPos) {
        turretPID.setPID(p, i, d);
        currentPos = turretEnc.getCurrentPosition();
        turretPower = turretPID.calculate(currentPos, targetPos);
        turretPower = util.clamp(turretPower, -turretMaxPower, turretMaxPower);
        return turretPower;
    }

    public void setTurret(double target){
        setTurretPow(setTurretPID(target));
    }

    public double getCurrentPos(){
        return currentPos;
    }

    public double getTargetPos(){
        return turretTarget;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){
        if (useTurretPID){
            setTurret(turretTarget);
        } else {
            setTurretPow(0);
        }
    }

}
