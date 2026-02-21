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
    public Lights lights;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        spindex = new Spindex(map);
        spinner = new Spinner(map);
        clutch = new Clutch(map);
        lights = new Lights(map);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        spindex.toInit();
        clutch.toInit();
        spinner.toInit();
        lights.toInit();
    }

    @Override
    public void update(){
        spindex.update();
        clutch.update();
        spinner.update();
        lights.update();

        if (spindex.spindexStartingSpinning){
            spinner.overrideSpinIn();
        }

        if (spindex.isSpindexAtPos() && spindex.spindexStartingSpinning){
            spinner.overrideSpinZero();
        }

        if(spinner.frontInnerTripped){
            if (spindex.isSpindexAtPos()) {
                if(spindex.isFrontColorDistanceTripped()) {
                    if (spindex.getCurrentDirection().equals("forward")) {
                        if (spindex.getCurrentBall().equals("one")) {
                            if (spindex.ballList[0].equals("E")) {
                                spindex.updateIntookBall();
                            }
                        } else if (spindex.getCurrentBall().equals("two")) {
                            if (spindex.ballList[1].equals("E")) {
                                spindex.updateIntookBall();
                            }
                        } else if (spindex.getCurrentBall().equals("three")) {
                            if (spindex.ballList[2].equals("E")) {
                                spindex.updateIntookBall();
                            }
                        }
                    }
                }
            } else {
                spinner.frontInnerTripped = false;
            }
        }

        if(spinner.backInnerTripped){
            if (spindex.isSpindexAtPos()) {
                if(spindex.isBackColorDistanceTripped()) {
                    if (spindex.getCurrentDirection().equals("backward")) {
                        if (spindex.getCurrentBall().equals("one")) {
                            if (spindex.ballList[0].equals("E")) {
                                spindex.updateIntookBall();
                            }
                        } else if (spindex.getCurrentBall().equals("two")) {
                            if (spindex.ballList[1].equals("E")) {
                                spindex.updateIntookBall();
                            }
                        } else if (spindex.getCurrentBall().equals("three")) {
                            if (spindex.ballList[2].equals("E")) {
                                spindex.updateIntookBall();
                            }
                        }
                    }
                }
            } else {
                spinner.backInnerTripped = false;
            }
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

        if(spindex.ballOneChanged){
            spindex.ballOneChanged = false;
            if(spindex.ballList[0].equals("E")){
                lights.setFrontLight("clear");
            } else if(spindex.ballList[0].equals("B")){
                lights.setFrontLightAlliance();
            } else if(spindex.ballList[0].equals("P")){
                lights.setFrontLight("purple");
            } else if(spindex.ballList[0].equals("G")){
                lights.setFrontLight("green");
            }
        }

        if(spindex.ballTwoChanged){
            spindex.ballTwoChanged = false;
            if(spindex.ballList[1].equals("E")){
                lights.setMiddleLight("clear");
            } else if(spindex.ballList[1].equals("B")){
                lights.setMiddleLightAlliance();
            } else if(spindex.ballList[1].equals("P")){
                lights.setMiddleLight("purple");
            } else if(spindex.ballList[1].equals("G")){
                lights.setMiddleLight("green");
            }
        }

        if(spindex.ballThreeChanged){
            spindex.ballThreeChanged = false;
            if(spindex.ballList[2].equals("E")){
                lights.setBackLight("clear");
            } else if(spindex.ballList[2].equals("B")){
                lights.setBackLightAlliance();
            } else if(spindex.ballList[2].equals("P")){
                lights.setBackLight("purple");
            } else if(spindex.ballList[2].equals("G")){
                lights.setBackLight("green");
            }
        }
    }

}
