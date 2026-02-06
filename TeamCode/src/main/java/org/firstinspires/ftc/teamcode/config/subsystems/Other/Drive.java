package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Drive implements Subsystem {

    //---------------- Hardware ----------------
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Drive(HardwareMap map) {
        leftBack = map.get(DcMotor.class, "left_back");
        rightBack = map.get(DcMotor.class, "right_back");
        leftFront = map.get(DcMotor.class, "left_front");
        rightFront = map.get(DcMotor.class, "right_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
