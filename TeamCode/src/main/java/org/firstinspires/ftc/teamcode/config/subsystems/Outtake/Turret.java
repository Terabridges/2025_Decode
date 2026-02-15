package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
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
public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private Servo leftTurret;
    private Servo rightTurret;
    private AnalogInput turretAnalog;
    private AbsoluteAnalogEncoder turretEnc;
    private Util util;


    //---------------- Software ----------------
    private double turretPos = 0;
    private double turretDegree = turretPos*360;
    public static double turretMinDeg = 50.0;
    public static double turretMaxDeg = 330.0;
    private double startTurret = 250;
    public static double turretVelocity = 0;
    public static double velocityLoopTime = 250;
    public static double visionKp = 0.8;
    public static double visionDeadbandDeg = 0.25;
    public static double visionMaxStepDeg = 5.0;
    public static double cameraLateralOffsetIn = 0.0;
    public static double visionDirection = -1.0; // set to 1.0 to invert lock direction
    public static double visionErrorBiasDeg = 0.0; // trim constant for steady left/right lock bias
    public static double limitAssistMarginDeg = 1.0;
    private double currentPosition = 0;
    private ElapsedTime velocityTimer;


    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(Servo.class, "turretL");
        rightTurret = map.get(Servo.class, "turretR");
        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        velocityTimer = new ElapsedTime();
        util = new Util();
    }

    //---------------- Methods ----------------
    public void setTurretPos(double pos){
        rightTurret.setPosition(util.clamp(pos, 0.0, 1.0));
    }

    public void setTurretDegree(double degree){
        setTurretPos(clampToSafeRange(normalizeDegrees(degree)) / 360.0);
    }

    public double normalizeDegrees(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    /**
     * Restrict turret commands to safe wiring range [turretMinDeg, turretMaxDeg].
     */
    public double clampToSafeRange(double degrees) {
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        return util.clamp(degrees, minDeg, maxDeg);
    }

    public boolean atMinLimit(double marginDeg) {
        return getCurrentDegrees() <= (Math.min(turretMinDeg, turretMaxDeg) + marginDeg);
    }

    public boolean atMaxLimit(double marginDeg) {
        return getCurrentDegrees() >= (Math.max(turretMinDeg, turretMaxDeg) - marginDeg);
    }

    public double computeParallaxCorrectionDeg(double distanceIn) {
        double safeDistance = Math.max(1.0, distanceIn);
        return Math.toDegrees(Math.atan2(cameraLateralOffsetIn, safeDistance));
    }

    public double computeVisionCorrectionDeg(double txDeg, double distanceIn) {
        double parallaxDeg = computeParallaxCorrectionDeg(distanceIn) * Math.signum(txDeg);
        return (txDeg + parallaxDeg + visionErrorBiasDeg) * visionKp * visionDirection;
    }

    /**
     * True when vision asks to move further outward while already at a safe turret limit.
     */
    public boolean needsChassisYawAssist(double txDeg, double distanceIn) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) {
            return false;
        }
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        return (atMinLimit(limitAssistMarginDeg) && correctionDeg < 0.0)
                || (atMaxLimit(limitAssistMarginDeg) && correctionDeg > 0.0);
    }

    /**
     * Aim turret at a field point using robot field pose.
     */
    public void aimAtFieldPoint(double robotX, double robotY, double robotHeadingRad,
                                double targetX, double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double headingToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        double turretDeg = normalizeDegrees(headingToTargetDeg - robotHeadingDeg);
        setTurretDegree(turretDeg);
    }

    /**
     * Vision lock controller for positional-servo turret.
     * Applies an incremental correction from tx (+ optional parallax correction from distance).
     */
    public void aimFromVision(double txDeg, double distanceIn) {
        if (Math.abs(txDeg) <= visionDeadbandDeg) {
            return;
        }
        double correctionDeg = computeVisionCorrectionDeg(txDeg, distanceIn);
        correctionDeg = util.clamp(correctionDeg, -visionMaxStepDeg, visionMaxStepDeg);
        double targetDeg = normalizeDegrees(getCurrentDegrees() + correctionDeg);
        setTurretDegree(targetDeg);
    }

    public void setTurretWithVelocity(){
        if (turretVelocity != 0 && velocityTimer.milliseconds() >= velocityLoopTime) {
            // Velocity nudges are also bounded by the same soft limits.
            setTurretDegree(getCurrentDegrees() + turretVelocity);
            velocityTimer.reset();
        }
    }

    public double getCurrentDegrees(){
        return rightTurret.getPosition()*360;
    }

    public double getVelocityTimerMs(){
        return velocityTimer.milliseconds();
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        velocityTimer.reset();
        setTurretDegree(startTurret);
    }

    @Override
    public void update(){
        setTurretWithVelocity();
    }

}
