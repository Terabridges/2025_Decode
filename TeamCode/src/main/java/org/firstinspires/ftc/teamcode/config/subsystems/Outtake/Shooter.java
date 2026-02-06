package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Shooter implements Subsystem {

    //---------------- Hardware ----------------
    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;
    private Servo hood;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Shooter(HardwareMap map) {
        leftFlywheel = map.get(DcMotor.class, "fly_left");
        rightFlywheel = map.get(DcMotor.class, "fly_right");
        hood = map.get(Servo.class, "hood");
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
