package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.psilynx.psikit.core.Logger;

/**
 * Hardware abstraction for the turret: two CRServos (geared together, same direction)
 * and an absolute analog encoder for position feedback.
 *
 * Provides calibrated turret-degree position, velocity estimation via a ring buffer
 * of encoder samples, and unified power output to both servos.
 */
@Configurable
public class TurretHardware {
    private static final String LOG_PREFIX = "Turret/Hardware/";
    private static final int VELOCITY_BUFFER_SIZE = 5;

    //---------------- Hardware ----------------
    private CRServo servoL;
    private CRServo servoR;
    private AnalogInput analogInput;
    private AbsoluteAnalogEncoder encoder;

    //---------------- Calibration (Configurable) ----------------
    /** Encoder reading (deg) at the reference turret position. */
    public static double encoderRefDeg = 201.8;
    /** True turret angle (deg) at the reference encoder position. */
    public static double encoderRefTurretDeg = 163.0;
    /** Scale factor: turret-degrees per encoder-degree near the reference point. */
    public static double encoderToTurretScale = 1.030;
    /** Set true if encoder direction is opposite to turret direction. */
    public static boolean encoderDirectionInverted = false;

    //---------------- Soft Limits (Configurable) ----------------
    /** Minimum allowed turret angle (deg). Power toward this limit is blocked. */
    public static double softLimitMinDeg = 30.0;
    /** Maximum allowed turret angle (deg). Power toward this limit is blocked. */
    public static double softLimitMaxDeg = 280.0;
    /** Margin from soft limit where power starts being attenuated (deg). */
    public static double softLimitMarginDeg = 15.0;
    /** Enable soft limit enforcement in setPower(). */
    public static boolean softLimitsEnabled = true;

    //---------------- Velocity Estimation ----------------
    private final double[] positionBuffer = new double[VELOCITY_BUFFER_SIZE];
    private final double[] timestampBuffer = new double[VELOCITY_BUFFER_SIZE];
    private int bufferIndex = 0;
    private int bufferCount = 0;
    private final ElapsedTime timer = new ElapsedTime();

    //---------------- State ----------------
    private double lastPower = 0.0;
    private double currentPositionDeg = 0.0;
    private double currentVelocityDegPerSec = 0.0;

    //---------------- Constructor ----------------
    public TurretHardware(HardwareMap map) {
        servoL = map.get(CRServo.class, "turretL");
        servoR = map.get(CRServo.class, "turretR");
        analogInput = map.get(AnalogInput.class, "turretAnalog");
        encoder = new AbsoluteAnalogEncoder(analogInput, 3.3, 0, 1);
        timer.reset();
    }

    //---------------- Core Methods ----------------

    /**
     * Send identical power to both turret CRServos.
     * If soft limits are enabled, power toward a limit is attenuated or blocked.
     * @param power value in [-1.0, 1.0]; 0 = stop
     */
    public void setPower(double power) {
        double clampedPower = Math.max(-1.0, Math.min(1.0, power));

        if (softLimitsEnabled) {
            double pos = currentPositionDeg;
            // Block power that would push further past a limit
            if (pos <= softLimitMinDeg && clampedPower < 0) {
                clampedPower = 0.0;
            } else if (pos >= softLimitMaxDeg && clampedPower > 0) {
                clampedPower = 0.0;
            }
            // Attenuate power in the margin zone
            if (clampedPower < 0 && pos < (softLimitMinDeg + softLimitMarginDeg)) {
                double distFromLimit = pos - softLimitMinDeg;
                double scale = Math.max(0.0, distFromLimit / softLimitMarginDeg);
                clampedPower *= scale;
            } else if (clampedPower > 0 && pos > (softLimitMaxDeg - softLimitMarginDeg)) {
                double distFromLimit = softLimitMaxDeg - pos;
                double scale = Math.max(0.0, distFromLimit / softLimitMarginDeg);
                clampedPower *= scale;
            }
        }

        lastPower = clampedPower;
        servoL.setPower(lastPower);
        servoR.setPower(lastPower);
    }

    /**
     * Send power bypassing soft limits (for calibration OpModes that know what they're doing).
     * @param power value in [-1.0, 1.0]; 0 = stop
     */
    public void setPowerRaw(double power) {
        lastPower = Math.max(-1.0, Math.min(1.0, power));
        servoL.setPower(lastPower);
        servoR.setPower(lastPower);
    }

    /** Returns true if turret position is at or past the soft limit in the given direction. */
    public boolean isAtSoftLimit(boolean positiveDirection) {
        if (!softLimitsEnabled) return false;
        return positiveDirection
                ? (currentPositionDeg >= softLimitMaxDeg)
                : (currentPositionDeg <= softLimitMinDeg);
    }

    /** Returns true if turret position is within the margin of a soft limit. */
    public boolean isNearSoftLimit() {
        if (!softLimitsEnabled) return false;
        return (currentPositionDeg <= (softLimitMinDeg + softLimitMarginDeg))
                || (currentPositionDeg >= (softLimitMaxDeg - softLimitMarginDeg));
    }

    /**
     * Must be called once per loop, before reading position/velocity.
     * Samples the encoder and updates the velocity ring buffer.
     */
    public void update() {
        double rawEncDeg = encoder.getCurrentPosition();
        currentPositionDeg = mapEncoderToTurretDeg(rawEncDeg);
        double nowSec = timer.seconds();

        // Store in ring buffer
        positionBuffer[bufferIndex] = currentPositionDeg;
        timestampBuffer[bufferIndex] = nowSec;
        bufferIndex = (bufferIndex + 1) % VELOCITY_BUFFER_SIZE;
        if (bufferCount < VELOCITY_BUFFER_SIZE) {
            bufferCount++;
        }

        // Compute velocity via linear regression over the ring buffer
        currentVelocityDegPerSec = computeVelocity();

        // Log hardware state
        Logger.recordOutput(LOG_PREFIX + "RawEncoderDeg", rawEncDeg);
        Logger.recordOutput(LOG_PREFIX + "MappedPositionDeg", currentPositionDeg);
        Logger.recordOutput(LOG_PREFIX + "VelocityDegPerSec", currentVelocityDegPerSec);
        Logger.recordOutput(LOG_PREFIX + "EncoderVoltage", analogInput.getVoltage());
        Logger.recordOutput(LOG_PREFIX + "CommandedPower", lastPower);
    }

    /**
     * Returns the turret position in degrees, mapped from the analog encoder
     * using the calibration reference point and scale.
     */
    public double getPositionDeg() {
        return currentPositionDeg;
    }

    /**
     * Returns the estimated turret angular velocity in degrees/second,
     * computed from a ring buffer of recent encoder samples using linear regression.
     */
    public double getVelocityDegPerSec() {
        return currentVelocityDegPerSec;
    }

    /** Raw encoder angle in degrees (0–360). */
    public double getEncoderRawDeg() {
        return encoder.getCurrentPosition();
    }

    /** Raw analog encoder voltage. */
    public double getEncoderVoltage() {
        return analogInput.getVoltage();
    }

    /** Last power sent to the servos. */
    public double getLastPower() {
        return lastPower;
    }

    //---------------- Encoder Mapping ----------------

    /**
     * Convert raw encoder degrees to calibrated turret degrees using the
     * two-point reference system (encoderRefDeg, encoderRefTurretDeg, encoderToTurretScale).
     */
    public double mapEncoderToTurretDeg(double encoderDeg) {
        if (Double.isNaN(encoderDeg) || Double.isInfinite(encoderDeg)) {
            return Double.NaN;
        }
        double deltaEncDeg = wrapSigned(encoderDeg - encoderRefDeg);
        if (encoderDirectionInverted) {
            deltaEncDeg = -deltaEncDeg;
        }
        double scale = (Math.abs(encoderToTurretScale) < 1e-6) ? 1.0 : encoderToTurretScale;
        return normalize(encoderRefTurretDeg + (deltaEncDeg * scale));
    }

    //---------------- Velocity Estimation ----------------

    /**
     * Compute velocity from the ring buffer using least-squares linear regression
     * of (timestamp, unwrapped position) pairs. Handles angle wrapping by unwrapping
     * relative to the first sample in the buffer.
     */
    private double computeVelocity() {
        if (bufferCount < 2) {
            return 0.0;
        }

        int oldest = (bufferCount < VELOCITY_BUFFER_SIZE) ? 0 : bufferIndex;
        double refPos = positionBuffer[oldest];

        // Compute means
        double sumT = 0, sumP = 0;
        for (int i = 0; i < bufferCount; i++) {
            int idx = (oldest + i) % VELOCITY_BUFFER_SIZE;
            sumT += timestampBuffer[idx];
            sumP += unwrap(positionBuffer[idx], refPos);
        }
        double meanT = sumT / bufferCount;
        double meanP = sumP / bufferCount;

        // Compute slope via least-squares
        double num = 0, den = 0;
        for (int i = 0; i < bufferCount; i++) {
            int idx = (oldest + i) % VELOCITY_BUFFER_SIZE;
            double dt = timestampBuffer[idx] - meanT;
            double dp = unwrap(positionBuffer[idx], refPos) - meanP;
            num += dt * dp;
            den += dt * dt;
        }

        if (Math.abs(den) < 1e-12) {
            return 0.0;
        }
        return num / den; // deg/sec
    }

    //---------------- Utility ----------------

    /** Normalize angle to [0, 360). */
    public static double normalize(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    /** Wrap angle to [-180, 180). */
    public static double wrapSigned(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /**
     * Unwrap a position relative to a reference so that the delta is in [-180, 180).
     * Returns the unwrapped absolute value (reference + signed delta).
     */
    private static double unwrap(double pos, double ref) {
        return ref + wrapSigned(pos - ref);
    }
}
