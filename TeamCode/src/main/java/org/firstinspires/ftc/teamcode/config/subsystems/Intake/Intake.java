package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class Intake implements Subsystem {

    //---------------- Hardware ----------------
    public Spindex spindex;
    public Spinner spinner;
    public Clutch clutch;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        spindex = new Spindex(map);
        spinner = new Spinner(map);
        clutch = new Clutch(map);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        spindex.toInit();
        clutch.toInit();
        spinner.toInit();
    }

    @Override
    public void update(){
        spindex.update();
        clutch.update();
        spinner.update();

        if (spindex.spindexStartingSpinning){
            spinner.overrideSpinIn();
        }

        if (spindex.isSpindexAtPos() && spindex.spindexStartingSpinning){
            spinner.overrideSpinZero();
        }

        if(spinner.frontInnerTripped){
            if (spindex.getCurrentDirection().equals("forward")){
                if(spindex.getCurrentBall().equals("one")){
                    if (spindex.ballList[0].equals("E")){
                        spindex.updateIntookBall();
                    }
                } else if(spindex.getCurrentBall().equals("two")){
                    if (spindex.ballList[1].equals("E")){
                        spindex.updateIntookBall();
                    }
                } else if(spindex.getCurrentBall().equals("three")){
                    if (spindex.ballList[2].equals("E")){
                        spindex.updateIntookBall();
                    }
                }
            }
            spinner.frontInnerTripped = false;
        }

        if(spinner.backInnerTripped){
            if (spindex.getCurrentDirection().equals("backward")){
                if(spindex.getCurrentBall().equals("one")){
                    if (spindex.ballList[0].equals("E")){
                        spindex.updateIntookBall();
                    }
                } else if(spindex.getCurrentBall().equals("two")){
                    if (spindex.ballList[1].equals("E")){
                        spindex.updateIntookBall();
                    }
                } else if(spindex.getCurrentBall().equals("three")){
                    if (spindex.ballList[2].equals("E")){
                        spindex.updateIntookBall();
                    }
                }
            }
            spinner.backInnerTripped = false;
        }

        if(spinner.frontOuterTripped){
            if(spindex.getCurrentDirection().equals("backward")){
                spindex.switchSides();
            }
            spinner.frontOuterTripped = false;
        }

        if(spinner.backOuterTripped){
            if(spindex.getCurrentDirection().equals("forward")){
                spindex.switchSides();
            }
            spinner.backOuterTripped = false;
        }

    }

}
