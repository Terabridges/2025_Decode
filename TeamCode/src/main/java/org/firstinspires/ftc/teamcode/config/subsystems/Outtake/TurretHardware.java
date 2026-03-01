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
    public static double softLimitMinDeg = 50.0;
    /** Maximum allowed turret angle (deg). Power toward this limit is blocked. */
    public static double softLimitMaxDeg = 260.0;
    /** Margin from soft limit where power starts being attenuated (deg). */
    public static double softLimitMarginDeg = 25.0;
    /** Enable soft limit enforcement in setPower(). */
    public static boolean softLimitsEnabled = true;
    /** Physical minimum turret angle (deg). Readings below this are encoder-wrap artifacts. */
    public static double physicalMinDeg = 15.0;
    /** Physical maximum turret angle (deg). Readings above this are encoder-wrap artifacts. */
    public static double physicalMaxDeg = 295.0;
    /** Emergency brake velocity threshold (deg/s). If speed toward a limit exceeds this, force stop. */
    public static double emergencyBrakeVelThreshold = 40.0;
    /**
     * Set true if positive CRServo power DECREASES turret angle.
     * When true, the hardware layer negates power at the servo write so that
     * all callers (OpModes, controllers) use the natural convention:
     * positive power = increasing turret angle.
     *
     * Confirmed via rlog analysis: positive CRServo power physically moves
     * the turret toward lower angles on this robot.
     */
    public static boolean servoPowerInverted = true;
    /** Emergency brake maximum reverse power magnitude (applied opposite to velocity). */
    public static double emergencyBrakePower = 0.20;
    /**
     * Velocity (deg/s) at which full emergency brake power is applied.
     * Below this, brake power scales linearly with velocity: brake = maxPower * (|vel| / fullVel).
     * Prevents overshoot bounce by tapering brake force as the turret decelerates.
     */
    public static double emergencyBrakeFullVelDps = 300.0;

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
    private double prevVelocityDegPerSec = 0.0; // for eBrake direction-change detection

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
     * <p>
     * Power is in the <b>turret-angle convention</b>: positive = increasing angle,
     * negative = decreasing angle. If {@link #servoPowerInverted} is set, the sign
     * is flipped before the hardware write so callers never need to think about it.
     * <p>
     * If soft limits are enabled, power toward a limit is attenuated or blocked.
     * @param power value in [-1.0, 1.0]; 0 = stop
     */
    public void setPower(double power) {
        double clampedPower = Math.max(-1.0, Math.min(1.0, power));
        double requestedPower = clampedPower; // save pre-limit value for logging
        String limitAction = "none";

        if (softLimitsEnabled) {
            double pos = currentPositionDeg;
            double vel = currentVelocityDegPerSec;
            boolean inMinZone = pos < (softLimitMinDeg + softLimitMarginDeg);
            boolean inMaxZone = pos > (softLimitMaxDeg - softLimitMarginDeg);

            // Emergency brake: proportional to velocity magnitude.
            // Harder brake when moving fast (dangerous), gentle when slow (prevents
            // overshoot bounce). Power tapers linearly: brake = maxPower * (|vel| / fullVel).
            // Once velocity reverses direction (brake worked), release immediately
            // to prevent launching the turret back the other way.
            if (inMinZone && vel < -emergencyBrakeVelThreshold) {
                // If velocity was positive last frame (moving away from min), the turret
                // has already reversed — don't brake, let it coast away.
                if (prevVelocityDegPerSec > 0) {
                    // Velocity just flipped back negative — could be oscillation.
                    // Just zero power, let attenuation handle it.
                    lastPower = 0.0;
                    writeServos(0.0);
                    logSoftLimits(requestedPower, 0.0, "eBrake_min_release", inMinZone, inMaxZone);
                    return;
                }
                double velMag = Math.abs(vel);
                double fraction = Math.min(1.0, velMag / emergencyBrakeFullVelDps);
                double brakePower = emergencyBrakePower * fraction; // positive = away from min
                lastPower = brakePower;
                writeServos(brakePower);
                logSoftLimits(requestedPower, brakePower, "eBrake_min", inMinZone, inMaxZone);
                return;
            }
            if (inMaxZone && vel > emergencyBrakeVelThreshold) {
                if (prevVelocityDegPerSec < 0) {
                    lastPower = 0.0;
                    writeServos(0.0);
                    logSoftLimits(requestedPower, 0.0, "eBrake_max_release", inMinZone, inMaxZone);
                    return;
                }
                double velMag = Math.abs(vel);
                double fraction = Math.min(1.0, velMag / emergencyBrakeFullVelDps);
                double brakePower = -emergencyBrakePower * fraction; // negative = away from max
                lastPower = brakePower;
                writeServos(brakePower);
                logSoftLimits(requestedPower, brakePower, "eBrake_max", inMinZone, inMaxZone);
                return;
            }

            // Block power that would push further past a limit
            // (positive power = toward max, negative = toward min — by convention)
            if (pos <= softLimitMinDeg && clampedPower < 0) {
                clampedPower = 0.0;
                limitAction = "block_min";
            } else if (pos >= softLimitMaxDeg && clampedPower > 0) {
                clampedPower = 0.0;
                limitAction = "block_max";
            }

            // Attenuate power in the margin zone (only toward the nearby limit)
            if (clampedPower < 0 && inMinZone) {
                double distFromLimit = pos - softLimitMinDeg;
                double scale = Math.max(0.0, distFromLimit / softLimitMarginDeg);
                clampedPower *= scale;
                limitAction = "atten_min(" + String.format("%.2f", scale) + ")";
            } else if (clampedPower > 0 && inMaxZone) {
                double distFromLimit = softLimitMaxDeg - pos;
                double scale = Math.max(0.0, distFromLimit / softLimitMarginDeg);
                clampedPower *= scale;
                limitAction = "atten_max(" + String.format("%.2f", scale) + ")";
            }

            logSoftLimits(requestedPower, clampedPower, limitAction, inMinZone, inMaxZone);
        }

        lastPower = clampedPower;
        writeServos(clampedPower);
    }

    /** Log soft limit diagnostics for AdvantageScope. */
    private void logSoftLimits(double requested, double actual, String action,
                               boolean inMinZone, boolean inMaxZone) {
        Logger.recordOutput(LOG_PREFIX + "SoftLimit/RequestedPower", requested);
        Logger.recordOutput(LOG_PREFIX + "SoftLimit/ActualPower", actual);
        Logger.recordOutput(LOG_PREFIX + "SoftLimit/Action", action);
        Logger.recordOutput(LOG_PREFIX + "SoftLimit/InMinZone", inMinZone);
        Logger.recordOutput(LOG_PREFIX + "SoftLimit/InMaxZone", inMaxZone);
    }

    /**
     * Send power bypassing soft limits (for calibration OpModes that know what they're doing).
     * Power uses the same turret-angle convention: positive = increasing angle.
     * @param power value in [-1.0, 1.0]; 0 = stop
     */
    public void setPowerRaw(double power) {
        lastPower = Math.max(-1.0, Math.min(1.0, power));
        writeServos(lastPower);
    }

    /**
     * Write power to both servos, applying direction inversion if needed.
     * This is the ONLY place that accounts for physical servo wiring polarity.
     */
    private void writeServos(double power) {
        double servoPower = servoPowerInverted ? -power : power;
        servoL.setPower(servoPower);
        servoR.setPower(servoPower);
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

    /** Returns true if turret is near the soft limit in the given travel direction. */
    public boolean isNearSoftLimit(boolean positiveDirection) {
        if (!softLimitsEnabled) return false;
        return positiveDirection
                ? (currentPositionDeg >= (softLimitMaxDeg - softLimitMarginDeg))
                : (currentPositionDeg <= (softLimitMinDeg + softLimitMarginDeg));
    }

    /**
     * Must be called once per loop, before reading position/velocity.
     * Samples the encoder and updates the velocity ring buffer.
     */
    public void update() {
        double rawEncDeg = encoder.getCurrentPosition();
        currentPositionDeg = mapEncoderToTurretDeg(rawEncDeg);

        // Clamp to physical range to prevent encoder 0/360° wrap artifacts.
        // The turret physically cannot reach positions outside [physicalMinDeg, physicalMaxDeg].
        // If the mapped angle lands outside that range, the encoder has wrapped
        // through its discontinuity — snap to the nearest physical limit.
        if (currentPositionDeg < physicalMinDeg || currentPositionDeg > physicalMaxDeg) {
            double distToMin = Math.abs(wrapSigned(currentPositionDeg - physicalMinDeg));
            double distToMax = Math.abs(wrapSigned(currentPositionDeg - physicalMaxDeg));
            currentPositionDeg = (distToMin <= distToMax) ? physicalMinDeg : physicalMaxDeg;
        }

        double nowSec = timer.seconds();

        // Store in ring buffer (clamped position — prevents velocity spikes from wrap)
        positionBuffer[bufferIndex] = currentPositionDeg;
        timestampBuffer[bufferIndex] = nowSec;
        bufferIndex = (bufferIndex + 1) % VELOCITY_BUFFER_SIZE;
        if (bufferCount < VELOCITY_BUFFER_SIZE) {
            bufferCount++;
        }

        // Compute velocity via linear regression over the ring buffer
        prevVelocityDegPerSec = currentVelocityDegPerSec;
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

    /**
     * Clamp a target angle to the safe operating range [softLimitMinDeg, softLimitMaxDeg].
     * Use this to validate any target before commanding a move.
     */
    public static double clampToSafeRange(double deg) {
        return Math.max(softLimitMinDeg, Math.min(softLimitMaxDeg, deg));
    }
}
