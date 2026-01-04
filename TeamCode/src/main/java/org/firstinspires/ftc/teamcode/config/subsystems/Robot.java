package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.psilynx.psikit.core.Logger;

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
        this(hardwareMap, telemetry, gp1, gp2, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2, boolean autoMode){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        vision = new Vision(hardwareMap);
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = autoMode ? new ShooterAuto(hardwareMap, vision) : new Shooter(hardwareMap, vision);
        transfer = new Transfer(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(drive, intake, shooter, transfer, vision));

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

    //---------------- Interface Methods ----------------
    public void update() {
        if (drive != null) {
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/RobotUpdate/Drive")) {
                drive.update();
            }
        }

        if (intake != null) {
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/RobotUpdate/Intake")) {
                intake.update();
            }
        }

        if (shooter != null) {
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/RobotUpdate/Shooter")) {
                shooter.update();
            }
        }

        if (transfer != null) {
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/RobotUpdate/Transfer")) {
                transfer.update();
            }
        }

        if (vision != null) {
            try (Logger.TimedBlock ignored = Logger.timeMs("LoggedRobot/UserSectionMS/RobotUpdate/Vision")) {
                vision.update();
            }
        }
    }

    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }
}
