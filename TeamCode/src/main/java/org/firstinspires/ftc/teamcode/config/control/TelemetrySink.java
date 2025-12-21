package org.firstinspires.ftc.teamcode.config.control;

/**
 * Desktop-safe telemetry sink.
 *
 * The FTC SDK's {@code Telemetry} isn't available on desktop, so controls write through this.
 */
public interface TelemetrySink {
    void addData(String caption, Object value);

    void addData(String caption, String format, Object... args);
}
