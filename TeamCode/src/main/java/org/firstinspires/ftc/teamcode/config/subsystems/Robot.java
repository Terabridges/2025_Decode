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

    //---------------- Objects ----------------
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public Gamepad gp1;
    public Gamepad gp2;
    public static VoltageSensor voltageSensor;

    //---------------- Subsystems ----------------
    public Shooter shooter;
    //public Limelight limelight;
    public List<Subsystem> subsystems;

    //---------------- Constructors ----------------
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        shooter = new Shooter(hardwareMap, telemetry);
        //limelight = new Limelight(hw, tele);

        subsystems = new ArrayList<>(Arrays.asList(shooter));

        this.gp1 = gp1;
        this.gp2 = gp2;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, null, null);
    }

    //---------------- Interface Methods ----------------
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
