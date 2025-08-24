package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {

    //Objects
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gp1;
    public Gamepad gp2;
    public static VoltageSensor voltageSensor;

    //Subsystems
    public Drive drive;
    public List<Subsystem> subsystems;

    //Constructors
    public Robot(HardwareMap map, Telemetry t, Gamepad gp1, Gamepad gp2){
        hardwareMap = map;
        telemetry = t;

        drive = new Drive(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(drive));

        this.gp1 = gp1;
        this.gp2 = gp2;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public Robot(HardwareMap map, Telemetry t){this(map, t, null, null);}

    //Methods


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