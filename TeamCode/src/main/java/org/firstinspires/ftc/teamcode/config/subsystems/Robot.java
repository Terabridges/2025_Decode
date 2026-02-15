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
        GO_TO_SHOOT_TWO,
        GO_TO_SHOOT_THREE,
        RESET
    }

    public boolean initShootAllMachine = false;

    //---------------- Subsystems ----------------

    public Intake intake;
    public Outtake outtake;
    public Other other;

    public List<org.firstinspires.ftc.teamcode.config.subsystems.Subsystem> subsystems;

    //---------------- Constructors ----------------
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2){
        this(hardwareMap, telemetry, gp1, gp2, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2, boolean autoMode){
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
        this(hardwareMap, telemetry, null, null, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean autoMode){
        this(hardwareMap, telemetry, null, null, autoMode);
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
                .transitionTimed(0.2, ShootAllStates.GO_TO_SHOOT_TWO)
                .onExit(()-> {
                    intake.clutch.setClutchDown();
                    intake.spindex.setSpindexShootOne();
                })

                .state(ShootAllStates.GO_TO_SHOOT_TWO)
                .transitionTimed(0.2, ShootAllStates.GO_TO_SHOOT_THREE)
                .onExit(()-> {
                    intake.spindex.setSpindexShootTwo();
                })

                .state(ShootAllStates.GO_TO_SHOOT_THREE)
                .transitionTimed(0.2, ShootAllStates.RESET)
                .onExit(()-> {
                    intake.spindex.setSpindexShootThree();
                })

                .state(ShootAllStates.RESET)
                .transitionTimed(0.2, ShootAllStates.INIT)
                .onExit(()-> {
                    intake.spindex.setSpindexForwardOne();
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.setClutchUp();
                    outtake.shooter.useFlywheelPID = false;
                })


                .build();

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
