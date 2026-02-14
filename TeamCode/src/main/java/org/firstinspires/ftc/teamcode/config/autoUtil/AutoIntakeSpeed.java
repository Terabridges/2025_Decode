package org.firstinspires.ftc.teamcode.config.autoUtil;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

public class AutoIntakeSpeed {
    private final double slope;
    private final double intercept;
    private final double minSpeed;
    private final double maxSpeed;
    private final double longRangeOffset;
    private final double redAllianceOffset;

    public AutoIntakeSpeed(double slope,
                           double intercept,
                           double minSpeed,
                           double maxSpeed,
                           double longRangeOffset,
                           double redAllianceOffset) {
        this.slope = slope;
        this.intercept = intercept;
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        this.longRangeOffset = longRangeOffset;
        this.redAllianceOffset = redAllianceOffset;
    }

    public double compute(double voltage, Alliance alliance, Range range) {
        double base = (slope * voltage) + intercept;
        double rangeOffset = (range == Range.LONG_RANGE) ? longRangeOffset : 0.0;
        double allianceOffset = (alliance == Alliance.RED) ? redAllianceOffset : 0.0;
        double speed = base + rangeOffset + allianceOffset;
        return clamp(speed, minSpeed, maxSpeed);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
