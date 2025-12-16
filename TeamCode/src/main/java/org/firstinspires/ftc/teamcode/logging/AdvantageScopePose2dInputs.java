package org.firstinspires.ftc.teamcode.logging;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;

/** Pose2d schema compatible with AdvantageScope (x/y/theta). Units: meters, radians. */
public final class AdvantageScopePose2dInputs implements LoggableInputs {

    private double xMeters;
    private double yMeters;
    private double thetaRad;

    public void set(double xMeters, double yMeters, double thetaRad) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.thetaRad = thetaRad;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("x", xMeters);
        table.put("y", yMeters);
        table.put("theta", thetaRad);
    }

    @Override
    public void fromLog(LogTable table) {
        xMeters = table.get("x", 0.0);
        yMeters = table.get("y", 0.0);
        thetaRad = table.get("theta", 0.0);
    }
}
