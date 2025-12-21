package org.firstinspires.ftc.teamcode.config.subsystems.io;

/**
 * Hardware abstraction for Drive.
 *
 * The goal is to keep all FTC SDK types out of the subsystem logic so it can be replayed on desktop.
 */
public interface DriveIO {

    final class Inputs {
        public int leftFrontPosTicks;
        public int rightFrontPosTicks;
        public int leftBackPosTicks;
        public int rightBackPosTicks;

        public double leftFrontVelTicksPerSec;
        public double rightFrontVelTicksPerSec;
        public double leftBackVelTicksPerSec;
        public double rightBackVelTicksPerSec;
    }

    /** Populate the provided inputs snapshot (may be left empty if not used yet). */
    void updateInputs(Inputs inputs);

    /** Apply motor powers in the standard mecanum order. */
    void setMotorPowers(double lf, double rf, double lb, double rb);
}
