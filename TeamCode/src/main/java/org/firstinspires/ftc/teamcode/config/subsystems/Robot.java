package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Other;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;

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
        RESET
    }

    public boolean initShootAllMachine = false;

    private double waitTime = 0.01;

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
                .transition(()-> initShootAllMachine, ShootAllStates.GO_TO_SHOOT_ONE)
                .onExit(()-> {
                    initShootAllMachine = false;
                    intake.spinner.setMegaSpinIn();
                    outtake.shooter.useFlywheelPID = true;
                    intake.spindex.setSpindexForwardOne();
                    intake.clutch.setClutchUp();
                })

                .state(ShootAllStates.GO_TO_SHOOT_ONE)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT0)
                .onExit(()-> {
                    intake.clutch.setClutchDown();
                    intake.spindex.setSpindexShootOne();
                })

                .state(ShootAllStates.WAIT0)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT1)

                .state(ShootAllStates.WAIT1)
                .transitionTimed(waitTime, ShootAllStates.GO_TO_SHOOT_TWO)
                .onExit(()-> {
                    intake.spindex.setSpindexShootTwo();
                })

                .state(ShootAllStates.GO_TO_SHOOT_TWO)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT2)

                .state(ShootAllStates.WAIT2)
                .transitionTimed(waitTime, ShootAllStates.GO_TO_SHOOT_THREE)
                .onExit(()-> {
                    intake.spindex.setSpindexShootThree();
                })

                .state(ShootAllStates.GO_TO_SHOOT_THREE)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT3)

                .state(ShootAllStates.WAIT3)
                .transitionTimed(waitTime, ShootAllStates.RESET)
                .onExit(()-> {
                    intake.spindex.setSpindexShootFour();
                })

                .state(ShootAllStates.RESET)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.INIT)
                .onExit(()-> {
                    intake.spindex.setSpindexForwardOne();
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.setClutchUp();
                    outtake.shooter.useFlywheelPID = false;
                })


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
