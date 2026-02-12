package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;
    public Vision vision;

    public List<Subsystem> subsystems;

    //---------------- Constructors ----------------
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        vision = new Vision(hardwareMap);
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);
        transfer = new Transfer(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(drive, intake, shooter, transfer, vision));

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
