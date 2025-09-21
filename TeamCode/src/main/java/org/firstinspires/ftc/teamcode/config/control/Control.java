package org.firstinspires.ftc.teamcode.config.control;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Control {
    void update();
    void addTelemetry(Telemetry t);
}
