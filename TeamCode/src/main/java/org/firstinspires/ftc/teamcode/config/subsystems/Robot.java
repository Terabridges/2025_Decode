package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;

public class Robot {
    public final Limelight ll = Limelight.INSTANCE;

    private final HardwareMap hw;
    private final Telemetry tele;

    public Robot(HardwareMap hw, Telemetry tele) {
        this.hw = hw;
        this.tele = tele;
    }

    public void init() {
        //Set device names
        ll.limelightName = "limelight";
        ll.imuName = "imu";

        // Bind + initialize each subsystem
        ll.bind(hw, tele);
        ll.initialize();
    }

    /** Tick all subsystems once per loop. */
    public void periodic() {
        ll.periodic();

        if (tele != null) tele.update();
    }
}
