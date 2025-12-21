package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import org.firstinspires.ftc.teamcode.config.subsystems.io.DriveIOFtc;

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
        drive = new Drive(new DriveIOFtc(hardwareMap));
        intake = new Intake(hardwareMap);
        shooter = autoMode ? new ShooterAuto(hardwareMap, vision) : new Shooter(hardwareMap, vision);
        transfer = new Transfer(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(drive, intake, shooter, transfer, vision));

        this.gp1 = gp1;
        this.gp2 = gp2;

        voltageSensor = findVoltageSensor(hardwareMap);
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, null, null, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean autoMode){
        this(hardwareMap, telemetry, null, null, autoMode);
    }

    public double getVoltage(){
        if (voltageSensor == null) {
            voltageSensor = findVoltageSensor(hardwareMap);
        }
        return voltageSensor.getVoltage();
    }

    private static VoltageSensor findVoltageSensor(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            return new MissingVoltageSensor();
        }

        try {
            List<VoltageSensor> sensors = hardwareMap.getAll(VoltageSensor.class);
            if (sensors != null && !sensors.isEmpty()) {
                return sensors.get(0);
            }
        } catch (Exception ignored) {
            // Some HardwareMap wrappers may throw here; fall through.
        }

        try {
            Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
            if (it != null && it.hasNext()) {
                return it.next();
            }
        } catch (Exception ignored) {
            // Fall through.
        }

        return new MissingVoltageSensor();
    }

    private static final class MissingVoltageSensor implements VoltageSensor {
        @Override
        public double getVoltage() {
            return 12.0;
        }

        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Unknown;
        }

        @Override
        public String getDeviceName() {
            return "MissingVoltageSensor";
        }

        @Override
        public String getConnectionInfo() {
            return "";
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
            // no-op
        }

        @Override
        public void close() {
            // no-op
        }
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
