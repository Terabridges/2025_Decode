package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.AbsoluteAnalogEncoder;

public class Shooter implements Subsystem{

    //---------------- Hardware ----------------
    public CRServo turret;
    public AnalogInput turretAnalog;
    public AbsoluteAnalogEncoder turretEnc;
    public DcMotor flyLeft;
    public DcMotor flyRight;
    public CRServo hood;
    public AnalogInput hoodAnalog;
    public AbsoluteAnalogEncoder hoodEnc;
    public TouchSensor hoodSwitch;

    //---------------- Software ----------------
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    double velocity = 0.0;
    private double desiredRpm = 0.0;
    boolean flyRun = false;

    //Auto Lock Stuff --------------------
    public boolean turretLock = false;
    private double kP = 0.020;       // tune on-bot
    private double kD = 0.001;       // damp overshoot
    private double maxTurretPower = 0.50;  // cap CR servo power
    private double deadbandDeg = 0.30;     // don't buzz near zero
    private double prevErrDeg = 0.0;
    // ------------------------------------


    //---------------- Constructor ----------------
    public Shooter(HardwareMap map) {
        turret = map.get(CRServo.class, "turret");
        flyLeft = map.get(DcMotor.class, "fly_left");
        flyRight = map.get(DcMotor.class, "fly_right");
        flyRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hood = map.get(CRServo.class, "hood");
        hoodSwitch = map.get(TouchSensor.class, "hood_switch");
        turretAnalog = map.get(AnalogInput.class, "turret_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        hoodAnalog = map.get(AnalogInput.class, "hood_analog");
        hoodEnc = new AbsoluteAnalogEncoder(hoodAnalog, 3.3, 0, 1);

        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //---------------- Methods ----------------
    public void turretLockUpdate(double txDeg) //pos tx turret too left, neg tx turret too right
    {
        if (turretLock) {
            double err = txDeg;                  // goal: tx -> 0
            double dErr = err - prevErrDeg;      // discrete derivative
            prevErrDeg = err;

            // small deadband to stop jitter at center
            if (Math.abs(err) < deadbandDeg) {
                setTurretPower(0.0);
                return;
            }

            double out = kP * err + kD * dErr;

            // clamp to safe CR servo range
            if (out > maxTurretPower) out = maxTurretPower;
            if (out < -maxTurretPower) out = -maxTurretPower;

            setTurretPower(out);
        }
    }

    private void setTurretPower(double pwr)
    {
        turret.setPower(pwr);
    }

    private double getTurretPos()
    {
        return turretEnc.getCurrentPosition();
    }

    public void toggleTurretLock()
    {
        turretLock = !turretLock;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
