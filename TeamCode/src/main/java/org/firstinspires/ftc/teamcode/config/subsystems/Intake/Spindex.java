package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;

@Configurable
public class Spindex implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo spindexLeft;
    private CRServo spindexRight;
    private AnalogInput spindexAnalog;
    private AbsoluteAnalogEncoder spindexEnc;
    private RevColorSensorV3 frontColor;
    private RevColorSensorV3 middleColor;
    private RevColorSensorV3 backColor;
    Util util;

    //---------------- Software ----------------
    public static double spindexTarget = 0.0;
    public static double currentPos = 0;
    private double spindexPower = 0.0;

//    public PIDController spindexPID;
//    public static double p = 0.0018, i = 0.00005, d = 0.000175; //for larger than one ball
//    public static double posTolerance = 5;
//    public static double integrationBounds = 5;
//    public static double spindexMaxPower = 0.75;
//    public static double thresh = 0.01;
//    public static double minPow = 0.08;

    public boolean useSpindexPID = true;
    private double frontBallOne = 30;
    private double ballForward = 60;
    private double switchIntakeForward = 35;

    public static double kPos = 0.012;      // position -> velocity gain (1/sec)
    public static  double kVel = 0.00035;    // velocity P (power per (ticks/sec))
    public static double kS   = 0.08;       // static friction feedforward (your minPow idea)
    public static double maxPower = 0.75;

    public static double aMax = 6000;       // ticks/sec^2  (tune)
    public static double omegaMax = 9000;   // ticks/sec    (tune)
    public static double posTol = 5;        // ticks
    public static double velTol = 50;       // ticks/sec

    // State for velocity estimate
    private double lastPos = 0.0;
    private boolean hasLast = false;
    private double omegaFilt = 0.0;
    public static double velFilterAlpha = 0.25; // 0..1 (higher = less filtering)

    ElapsedTime timer;


    //---------------- Constructor ----------------
    public Spindex(HardwareMap map) {
        spindexLeft = map.get(CRServo.class, "spindexL");
        spindexRight = map.get(CRServo.class, "spindexR");
        spindexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontColor = map.get(RevColorSensorV3.class, "color1");
        middleColor = map.get(RevColorSensorV3.class, "color2");
        backColor = map.get(RevColorSensorV3.class, "color3");
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 0, 1);

//        spindexPID = new PIDController(p, i, d);
//        spindexPID.setIntegrationBounds(-integrationBounds, integrationBounds);
//        spindexPID.setTolerance(posTolerance);
        util = new Util();
        timer = new ElapsedTime();
    }

    //---------------- Methods ----------------
    public void setSpindexPow(double pow){
        spindexLeft.setPower(pow);
        spindexRight.setPower(pow);
    }

    private static double sign(double x) {
        return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
    }

//    public double setSpindexPID(double targetPos) {
//        spindexPID.setPID(p, i, d);
//        currentPos = spindexEnc.getCurrentPosition();
//        spindexPower = spindexPID.calculate(currentPos, targetPos);
//        spindexPower = util.clamp(spindexPower, -spindexMaxPower, spindexMaxPower);
//        if (spindexPower > thresh){
//            if (spindexPower < minPow){
//                spindexPower = minPow;
//            }
//        } else if (spindexPower < -thresh){
//            if (spindexPower > -minPow){
//                spindexPower = -minPow;
//            }
//        }
//        return spindexPower;
//    }
//
//    public void setSpindex(double target){
//        setSpindexPow(setSpindexPID(target));
//    }
//
    public double getCurrentPosition(){
        return currentPos;
    }

    public double getTargetPosition(){
        return spindexTarget;
    }

    public double getSpindexPID(double pos, double target, double dt) {
        if (dt <= 1e-4) dt = 1e-4;

        // Velocity estimate (ticks/sec) with light filtering to avoid D-noise vibes
        double omega = 0.0;
        if (hasLast) omega = (pos - lastPos) / dt;
        lastPos = pos;
        hasLast = true;
        omegaFilt = omegaFilt + velFilterAlpha * (omega - omegaFilt);

        double error = target - pos;

        // Stop condition (prevents tiny oscillation near target)
        if (Math.abs(error) <= posTol && Math.abs(omegaFilt) <= velTol) {
            return 0.0;
        }

        // --- Braking speed limit ---
        // omegaLimit = sqrt(2 * aMax * |error|)
        double omegaLimit = Math.sqrt(2.0 * aMax * Math.abs(error));
        omegaLimit = Math.min(omegaLimit, omegaMax);

        // Position -> desired velocity, then cap it by braking limit
        double omegaCmd = kPos * error;                 // (ticks/sec)
        omegaCmd = util.clamp(omegaCmd, -omegaLimit, omegaLimit);

        // Velocity control (simple P) + static friction feedforward
        double velErr = omegaCmd - omegaFilt;
        double power = kVel * velErr;

        // Apply kS ONLY when we intend to move (prevents kick-induced oscillation)
        if (Math.abs(omegaCmd) > 1.0) {
            power += kS * sign(omegaCmd);
        }

        // Clamp output
        power = util.clamp(power, -maxPower, maxPower);
        spindexPower = power;
        return spindexPower;
    }

    public void setSpindex(){
        setSpindexPow(getSpindexPID(currentPos, spindexTarget, timer.seconds()));
    }


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        //spindexTarget = frontBallOne;
        timer.reset();
    }

    @Override
    public void update(){
        if (useSpindexPID){
            //setSpindex(spindexTarget);
            currentPos = spindexEnc.getCurrentPosition();
            setSpindex();

        } else {
            setSpindexPow(0);
        }
        timer.reset();
    }
}
