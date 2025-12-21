package org.firstinspires.ftc.teamcode.config.control.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.TelemetrySink;

/** Wraps FTC SDK {@link Telemetry} as a desktop-safe {@link TelemetrySink}. */
public final class FtcTelemetrySink implements TelemetrySink {

    private final Telemetry telemetry;

    public FtcTelemetrySink(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    @Override
    public void addData(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }
}
