package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.config.pedroPathing.constants.LConstants;

public class Drive implements Subsystem {

    //Hardware
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    //Software

    //Constructor
    public Drive(HardwareMap map) {
        leftBack = map.get(DcMotor.class, "left_back");
        leftFront = map.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = map.get(DcMotor.class, "right_back");
        rightFront = map.get(DcMotor.class, "right_front");
    }

    //Methods


    //Interface Methods
    @Override
    public void toInit(){
    }

    @Override
    public void update(){
    }
}