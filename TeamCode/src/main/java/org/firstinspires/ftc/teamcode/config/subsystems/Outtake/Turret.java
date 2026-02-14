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
    private double lowLimit = 50; //when spinning clockwise
    private double highLimit = 330; //when spinning counterclockwise
    private double startTurret = 250;
    public static double turretVelocity = 0;
    public static double velocityLoopTime = 250;
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
        rightTurret.setPosition(pos);
    }

    public void setTurretDegree(double degree){
        setTurretPos(degree/360);
    }

    public void setTurretWithVelocity(){
        if (turretVelocity > 0 && getCurrentDegrees()+turretVelocity > highLimit){
            setTurretDegree(lowLimit);
        } else if (turretVelocity < 0 && getCurrentDegrees()+turretVelocity < lowLimit){
            setTurretDegree(highLimit);
        }

        if (turretVelocity != 0 && velocityTimer.milliseconds() >= velocityLoopTime) {
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
