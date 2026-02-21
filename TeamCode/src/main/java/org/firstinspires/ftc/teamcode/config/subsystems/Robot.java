package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Other;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {

    //---------------- Objects ----------------
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private Gamepad gp1;
    private Gamepad gp2;
    private static VoltageSensor voltageSensor;

    public enum ShootAllStates {
        INIT,
        GO_TO_SHOOT_ONE,
        WAIT0,
        WAIT1,
        GO_TO_SHOOT_TWO,
        WAIT2,
        GO_TO_SHOOT_THREE,
        WAIT3,
        RESET,
        UNJAM
    }

    public enum SortedShootAllStates {
        INIT,
        GO_TO_FIRST,
        WAIT0,
        WAIT1,
        GO_TO_SECOND,
        WAIT2,
        GO_TO_THIRD,
        WAIT3,
        WAIT4,
        CLUTCH,
        RESET,
        UNJAM

    }

    public boolean initShootAllMachine = false;

    public boolean initSortedShootAllMachine = false;

    private double waitTime = 0.01;

    private boolean goToReset = false;
    private int shootAllBallTargetCount = 0;

    public boolean useSorting = false;


    //---------------- Subsystems ----------------

    public Intake intake;
    public Outtake outtake;
    public Other other;

    public List<org.firstinspires.ftc.teamcode.config.subsystems.Subsystem> subsystems;

    //---------------- Constructors ----------------
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        other = new Other(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(intake, outtake, other));

        this.gp1 = gp1;
        this.gp2 = gp2;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, null, null);
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public StateMachine getShootAllMachine(){
        return new StateMachineBuilder()
                .state(ShootAllStates.INIT)
                .transition(() -> initShootAllMachine && getLoadedBallCount() > 0, ShootAllStates.GO_TO_SHOOT_ONE)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)
                .onExit(()-> {
                    initShootAllMachine = false;
                    shootAllBallTargetCount = Math.max(0, Math.min(3, getLoadedBallCount()));
                    intake.spinner.setMegaSpinIn();
                    outtake.shooter.useFlywheelPID = true;
                    intake.spindex.setSpindexForwardZero();
                    intake.clutch.setClutchUp();
                    outtake.shooter.setHoodTarget();
                })

                .state(ShootAllStates.GO_TO_SHOOT_ONE)
                .transition(()-> intake.spindex.isSpindexAtPos() && outtake.shooter.isAtRPM(), ShootAllStates.WAIT0)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)
                .onExit(()-> {
                    intake.clutch.setClutchDown();
                    intake.spindex.setSpindexShootOne();
                })

                .state(ShootAllStates.WAIT0)
                .transition(() -> intake.spindex.isSpindexAtPos() && shootAllBallTargetCount <= 1, ShootAllStates.RESET)
                .transition(() -> intake.spindex.isSpindexAtPos() && shootAllBallTargetCount > 1, ShootAllStates.WAIT1)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)

                .state(ShootAllStates.WAIT1)
                .transitionTimed(waitTime, ShootAllStates.GO_TO_SHOOT_TWO)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)

                .state(ShootAllStates.GO_TO_SHOOT_TWO)
                .onEnter(() -> {
                    intake.spindex.setSpindexShootTwo();
                })
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT2)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)

                .state(ShootAllStates.WAIT2)
                .transitionTimed(waitTime, ShootAllStates.GO_TO_SHOOT_THREE)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)

                .state(ShootAllStates.GO_TO_SHOOT_THREE)
                .onEnter(() -> {
                    if (shootAllBallTargetCount > 2) {
                        intake.spindex.setSpindexShootThree();
                    }
                })
                .transition(() -> shootAllBallTargetCount <= 2, ShootAllStates.RESET)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT3)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)

                .state(ShootAllStates.WAIT3)
                .transitionTimed(waitTime, ShootAllStates.RESET)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)
                .onExit(()-> {
                    intake.spindex.setSpindexShootFour();
                })

                .state(ShootAllStates.RESET)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.INIT)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)
                .onExit(()-> {
                    shootAllBallTargetCount = 0;
                    intake.spindex.setSpindexForwardOne();
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.setClutchUp();
                    intake.spindex.emptyBalls();
                })

                .state(ShootAllStates.UNJAM)
                .onEnter(()->{
                    other.unJam = false;
                    shootAllBallTargetCount = 0;
                    intake.spindex.setSpindexPos(intake.spindex.getAbsolutePos());
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.setClutchUp();
                    intake.spindex.emptyBalls();
                    goToReset = true;
                })
                .transition(()-> goToReset, ShootAllStates.INIT)
                .onExit(()->goToReset = false)

                .build();
    }

    public int getLoadedBallCount() {
        if (intake == null || intake.spindex == null || intake.spindex.ballList == null) {
            return 0;
        }
        int count = 0;
        String[] ballList = intake.spindex.ballList;
        for (String slot : ballList) {
            if (slot != null && !slot.equals("E")) {
                count++;
            }
        }
        return count;
    }

    public StateMachine getSortedShootAllMachine(){
        double startBall = getBallSortedShootOrder();
        return new StateMachineBuilder()
                .state(SortedShootAllStates.INIT)
                .transition(()-> initSortedShootAllMachine, SortedShootAllStates.GO_TO_FIRST)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {
                    initSortedShootAllMachine = false;
                    intake.spinner.setMegaSpinIn();
                    outtake.shooter.useFlywheelPID = true;
                    intake.clutch.setClutchUp();
                    outtake.shooter.setHoodTarget();

                    if(startBall == 1){
                        intake.spindex.setSpindexForwardZero();
                    } else if(startBall == 2){
                        intake.spindex.setSpindexForwardOne();
                    } else if(startBall == 3){
                        intake.spindex.setSpindexForwardTwo();
                    }
                })

                .state(SortedShootAllStates.GO_TO_FIRST)
                .transition(()-> intake.spindex.isSpindexAtPos() && outtake.shooter.isAtRPM(), SortedShootAllStates.WAIT0)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {
                    intake.clutch.setClutchDown();

                    if(startBall == 1){
                        intake.spindex.setSpindexShootOne();
                    } else if(startBall == 2){
                        intake.spindex.setSpindexShootTwo();
                    } else if(startBall == 3){
                        intake.spindex.setSpindexShootThree();
                    }
                })

                .state(SortedShootAllStates.WAIT0)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.WAIT1)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)

                .state(SortedShootAllStates.WAIT1)
                .transitionTimed(waitTime, SortedShootAllStates.GO_TO_SECOND)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {

                    if(startBall == 1){
                        intake.spindex.setSpindexShootTwo();
                    } else if(startBall == 2){
                        intake.spindex.setSpindexShootThree();
                    } else if(startBall == 3){
                        intake.spindex.setSpindexShootFour();
                    }
                })

                .state(SortedShootAllStates.GO_TO_SECOND)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.WAIT2)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)

                .state(SortedShootAllStates.WAIT2)
                .transitionTimed(waitTime, SortedShootAllStates.GO_TO_THIRD)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {

                    if(startBall == 1){
                        intake.spindex.setSpindexShootThree();
                    } else if(startBall == 2){
                        intake.spindex.setSpindexShootFour();
                    } else if(startBall == 3){
                        intake.spindex.setSpindexShootFive();
                    }
                })

                .state(SortedShootAllStates.GO_TO_THIRD)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.WAIT3)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)

                .state(SortedShootAllStates.WAIT3)
                .transitionTimed(waitTime, SortedShootAllStates.RESET)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {
                    if(startBall == 1){
                        intake.spindex.setSpindexShootFour();
                    } else if(startBall == 2){
                        intake.spindex.setSpindexShootFive();
                    } else if(startBall == 3){
                        intake.spindex.setSpindexShootSix();
                    }
                })

                .state(ShootAllStates.RESET)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.INIT)
                .transition(()-> other.unJam, ShootAllStates.UNJAM)
                .onExit(()-> {
                    intake.spindex.setSpindexForwardOne();
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.setClutchUp();
                    intake.spindex.emptyBalls();
                })

                .state(SortedShootAllStates.UNJAM)
                .onEnter(()->{
                    other.unJam = false;
                    intake.spindex.setSpindexPos(intake.spindex.getAbsolutePos());
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.setClutchUp();
                    intake.spindex.emptyBalls();
                    goToReset = true;
                })
                .transition(()-> goToReset, SortedShootAllStates.INIT)
                .onExit(()->goToReset = false)

                .build();
    }

//    public StateMachine getShootAllMachine(){
//        return new StateMachineBuilder()
//                .state(ShootAllStates.INIT)
//                .transition(()-> initShootAllMachine, ShootAllStates.GO_TO_SHOOT_ONE)
//                .onExit(()-> {
//                    initShootAllMachine = false;
//                    intake.spinner.setMegaSpinIn();
//                    outtake.shooter.useFlywheelPID = true;
//                    intake.spindex.setSpindexForwardOne();
//                    intake.clutch.setClutchUp();
//                })
//
//                .state(ShootAllStates.GO_TO_SHOOT_ONE)
//                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.RESET)
//                .onExit(()-> {
//                    intake.clutch.setClutchDown();
//                    intake.spindex.setSpindexShootThree();
//                })
//
//                .state(ShootAllStates.RESET)
//                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.INIT)
//                .onExit(()-> {
//                    intake.spinner.setMegaSpinZero();
//                    intake.clutch.setClutchUp();
//                    outtake.shooter.useFlywheelPID = false;
//                })
//
//                .build();
//
//    }

    public int getBallSortedShootOrder(){
        if(GlobalVariables.getMotif().equals(GlobalVariables.MotifPattern.PPG)) {
            if (intake.spindex.balls.equals("PPG")){
                return 1;
            } else if (intake.spindex.balls.equals("GPP")){
                return 2;
            } else if (intake.spindex.balls.equals("PGP")){
                return 3;
            } else {
                return 1;
            }
        } else if(GlobalVariables.getMotif().equals(GlobalVariables.MotifPattern.GPP)) {
            if (intake.spindex.balls.equals("GPP")){
                return 1;
            } else if (intake.spindex.balls.equals("PGP")){
                return 2;
            } else if (intake.spindex.balls.equals("PPG")){
                return 3;
            } else {
                return 1;
            }
        } else if(GlobalVariables.getMotif().equals(GlobalVariables.MotifPattern.PGP)) {
            if (intake.spindex.balls.equals("PGP")){
                return 1;
            } else if (intake.spindex.balls.equals("PPG")){
                return 2;
            } else if (intake.spindex.balls.equals("GPP")){
                return 3;
            } else {
                return 1;
            }
        } else {
            return 1;
        }
    }

    public void getReadyShoot(){
        if (useSorting){
            int firstBall = getBallSortedShootOrder();
            if (firstBall == 1){
                intake.spindex.setSpindexForwardZero();
            } else if (firstBall == 2){
                intake.spindex.setSpindexForwardOne();
            } else if (firstBall == 3) {
                intake.spindex.setSpindexForwardTwo();
            }
        } else {
            intake.spindex.setSpindexForwardZero();
        }
        outtake.shooter.setHoodTarget();
    }

    public void toggleSorting(){
        useSorting = !useSorting;
    }

    //---------------- Interface Methods ----------------
    public void update() {
        for (org.firstinspires.ftc.teamcode.config.subsystems.Subsystem s : subsystems) {
            s.update();
        }
    }

    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }
}
