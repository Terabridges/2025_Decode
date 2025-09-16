package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {

    //Objects
    private final HardwareMap hw;
    private final Telemetry tele;
    public Gamepad gp1;
    public Gamepad gp2;

    //public static VoltageSensor voltageSensor;

    //Subsystems

    public Shooter shooter;
    public Limelight limelight;

    //Other
    public String currentState = "none";

    //Subsystem List
    public List<Subsystem> subsystems;

    public Robot(HardwareMap hw, Telemetry tele, Gamepad gp1, Gamepad gp2){
        this.hw = hw;
        this.tele = tele;

        shooter = new Shooter(hw, tele);
        limelight = new Limelight(hw, tele);

        subsystems = new ArrayList<>(Arrays.asList(shooter, limelight));

        this.gp1 = gp1;
        this.gp2 = gp2;

        //voltageSensor = hw.voltageSensor.iterator().next();
    }

    //For Auto
    public Robot(HardwareMap hw, Telemetry tele){
        this.hw = hw;
        this.tele = tele;

        shooter = new Shooter(hw, tele);
        limelight = new Limelight(hw, tele);

        subsystems = new ArrayList<>(Arrays.asList(shooter, limelight));

        //voltageSensor = hw.voltageSensor.iterator().next();
    }

    //Interface Methods
    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }

    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }
}
