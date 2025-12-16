package org.firstinspires.ftc.teamcode.logging;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;

/**
 * Pose3d schema compatible with AdvantageScope.
 *
 * <p>Translation units: meters. Quaternion: (w, x, y, z).
 */
public final class AdvantageScopePose3dInputs implements LoggableInputs {

    private double xMeters;
    private double yMeters;
    private double zMeters;

    private double qw;
    private double qx;
    private double qy;
    private double qz;

    public void setTranslation(double xMeters, double yMeters, double zMeters) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.zMeters = zMeters;
    }

    public void setQuaternion(double qw, double qx, double qy, double qz) {
        this.qw = qw;
        this.qx = qx;
        this.qy = qy;
        this.qz = qz;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("translation/x", xMeters);
        table.put("translation/y", yMeters);
        table.put("translation/z", zMeters);

        table.put("rotation/qw", qw);
        table.put("rotation/qx", qx);
        table.put("rotation/qy", qy);
        table.put("rotation/qz", qz);
    }

    @Override
    public void fromLog(LogTable table) {
        xMeters = table.get("translation/x", 0.0);
        yMeters = table.get("translation/y", 0.0);
        zMeters = table.get("translation/z", 0.0);

        qw = table.get("rotation/qw", 1.0);
        qx = table.get("rotation/qx", 0.0);
        qy = table.get("rotation/qy", 0.0);
        qz = table.get("rotation/qz", 0.0);
    }
}
