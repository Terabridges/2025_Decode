package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;

@Configurable
public class Spindex implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo spindexLeft;
    private CRServo spindexRight;
    private AnalogInput spindexAnalog;
    private AbsoluteAnalogEncoder spindexEnc;
    private RevColorSensorV3 frontColor;
    private RevColorSensorV3 middleColor;
    private RevColorSensorV3 backColor;
    Util util;

    //---------------- Software ----------------
    public static double spindexTarget = 0.0; // RAW target in [0, 360)
    public static double currentPos = 0;      // UNWRAPPED current position (continuous degrees)
    private double spindexPower = 0.0;

    // Manual mode support (used by tuning opmodes). If useSpindexPID == false, update() applies this power.
    public static double manualPower = 0.0;

    public boolean useSpindexPID = true;
    private double frontBallOne = 30;
    private double ballForward = 60;
    private double switchIntakeForward = 35;

    public static double kPos = 1.0; // 0.012;      // position -> velocity gain (1/sec)
    public static double kVel = 0.001; //0.00035;    // velocity P (power per (deg/sec))
    public static double kS   = 0.08;       // static friction feedforward (legacy; prefer kSPos/kSNeg)
    public static double kSPos = .126 ; //0.079;      // static friction feedforward for + direction
    public static double kSNeg = .097 ; //0.065;      // static friction feedforward for - direction
    public static double maxPower = 0.35; // 0.75;

    // kS application shaping (prevents kS from sustaining oscillations once moving)
    public static double kSErrorFullDeg = 12; // deg error where kS is fully applied
    public static double kSOmegaFullDegPerSec = 25; // deg/s below which kS is fully applied

    // Output limiting (helps when motors are geared for speed)
    public static double powerSlewPerSec = 8.0; // power units per second (0 disables)
    private double lastOutPower = 0.0;

    // Small integral hold to eliminate steady-state error (off by default)
    // Integrates only near target and when moving slowly.
    public static double kIHold = 0.006;               // power per (deg * sec)
    public static double iZoneDeg = 25.0;              // only integrate when |error| <= this
    public static double iZoneOmegaDegPerSec = 150.0;  // only integrate when |omegaFilt| <= this
    public static double iMaxPower = 0.12;             // clamp on integral contribution
    public static double iLeakPerSec = 0.6;            // decay toward 0 when not integrating
    private double iPower = 0.0;
    private double lastIError = 0.0;
    private boolean lastIInZone = false;
    private boolean lastISaturated = false;

    public static double aMax = 1500; //6000;       // deg/sec^2  (tune)
    public static double omegaMax = 600; // 9000;   // deg/sec    (tune)
    public static double posTol = 5;        // deg (used for kS gating / general "close enough")
    public static double velTol = 50;       // deg/sec (used for kS gating / general "close enough")

    // Separate stop thresholds (when met, output is forced to 0).
    // Default matches prior behavior; tune stopTolDeg lower if you want smaller steady-state error.
    public static double stopTolDeg = 1; // 5;
    public static double stopVelTolDegPerSec = 25;

    // Optional position filtering for noisy analog signals (1.0 = no filtering)
    public static double posFilterAlpha = 0.2; //1.0; // 0..1
    private double posFilt = 0.0;
    private boolean hasPosFilt = false;

    // State for velocity estimate
    private double lastPos = 0.0;      // UNWRAPPED pos last loop
    private boolean hasLast = false;
    private double omegaFilt = 0.0;
    public static double velFilterAlpha = 0.1; // 0.25; // 0..1 (higher = less filtering)

    // Absolute analog encoder calibration (degrees)
    public static double encOffsetDeg = 0.0;
    public static boolean encInverted = false;

    // Unwrap state for absolute encoder (0..360)
    private double unwrappedPos = 0.0;
    private double lastRawPos = 0.0;
    private boolean hasRaw = false;
    private static final double WRAP_SIZE_DEG = 360.0;

    // For debug/telemetry if you want it later
    private double targetPosUnwrapped = 0.0;

    // Debug/telemetry state from last control update
    private double lastDt = 0.0;
    private double lastRawDeg = 0.0;
    private double lastVoltage = 0.0;
    private double lastError = 0.0;
    private double lastOmega = 0.0;
    private double lastOmegaCmd = 0.0;
    private double lastOmegaLimit = 0.0;
    private double lastVelErr = 0.0;

    ElapsedTime timer;

    //---------------- Constructor ----------------
    public Spindex(HardwareMap map) {
        this(map, true);
    }

    public Spindex(HardwareMap map, boolean initColorSensors) {
        spindexLeft = map.get(CRServo.class, "spindexL");
        spindexRight = map.get(CRServo.class, "spindexR");
        spindexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (initColorSensors) {
            frontColor = map.get(RevColorSensorV3.class, "color1");
            middleColor = map.get(RevColorSensorV3.class, "color2");
            backColor = map.get(RevColorSensorV3.class, "color3");
        }
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 0, 1);
        spindexEnc.zero(encOffsetDeg).setInverted(encInverted);

        util = new Util();
        timer = new ElapsedTime();
    }

    //---------------- Methods ----------------
    public void setSpindexPow(double pow){
        spindexLeft.setPower(pow);
        spindexRight.setPower(pow);
    }

    private static double sign(double x) {
        return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
    }

    /**
     * Unwrap absolute encoder angle (0..360) into a continuous angle.
     */
    private double unwrapDegrees(double rawDeg) {
        if (!hasRaw) {
            hasRaw = true;
            lastRawPos = rawDeg;
            unwrappedPos = rawDeg;
            return unwrappedPos;
        }

        double delta = rawDeg - lastRawPos;

        // Wrap delta into (-180, 180]
        if (delta >  WRAP_SIZE_DEG / 2.0) delta -= WRAP_SIZE_DEG;
        if (delta < -WRAP_SIZE_DEG / 2.0) delta += WRAP_SIZE_DEG;

        unwrappedPos += delta;
        lastRawPos = rawDeg;
        return unwrappedPos;
    }

    /**
     * Convert a raw target in [0,360) to the nearest equivalent angle to the current unwrapped position.
     * Example: current = 725 deg, targetRaw = 10 deg -> nearest is 730 deg (not 10 deg).
     */
    private double nearestTargetUnwrapped(double currentUnwrappedDeg, double targetRawDeg) {
        double k = Math.round((currentUnwrappedDeg - targetRawDeg) / WRAP_SIZE_DEG);
        return targetRawDeg + k * WRAP_SIZE_DEG;
    }

    public double getCurrentPosition(){
        return currentPos;
    }

    public double getCurrentPositionFiltered() {
        return posFilt;
    }

    public double getTargetPosition(){
        return spindexTarget; // raw 0..360 as you had
    }

    public double getTargetPositionUnwrapped() {
        return targetPosUnwrapped;
    }

    public double getLastPower() {
        return spindexPower;
    }

    public double getLastDt() {
        return lastDt;
    }

    public double getLastRawDeg() {
        return lastRawDeg;
    }

    public double getLastVoltage() {
        return lastVoltage;
    }

    public double getLastErrorDeg() {
        return lastError;
    }

    public double getLastOmegaDegPerSec() {
        return omegaFilt;
    }

    public double getLastOmegaUnfilteredDegPerSec() {
        return lastOmega;
    }

    public double getLastOmegaCmdDegPerSec() {
        return lastOmegaCmd;
    }

    public double getLastIPower() {
        return iPower;
    }

    public boolean getLastIInZone() {
        return lastIInZone;
    }

    public boolean getLastISaturated() {
        return lastISaturated;
    }

    public double getLastOmegaLimitDegPerSec() {
        return lastOmegaLimit;
    }

    public double getLastVelErrDegPerSec() {
        return lastVelErr;
    }

    /**
     * Braking-limited cascaded position->velocity controller.
     *
     * pos and target are in UNWRAPPED degrees.
     * dt is seconds.
     */
    public double getSpindexPID(double pos, double target, double dt) {
        if (dt <= 1e-4) dt = 1e-4;

        // Velocity estimate (deg/sec) with filtering
        double omega = 0.0;
        if (hasLast) omega = (pos - lastPos) / dt;
        lastPos = pos;
        hasLast = true;

        lastOmega = omega;

        omegaFilt = omegaFilt + velFilterAlpha * (omega - omegaFilt);

        double error = target - pos;
        lastError = error;

        // Stop condition (prevents tiny oscillation near target)
        if (Math.abs(error) <= stopTolDeg && Math.abs(omegaFilt) <= stopVelTolDegPerSec) {
            iPower = 0.0;
            lastIInZone = false;
            lastISaturated = false;
            spindexPower = 0.0;
            return 0.0;
        }

        // --- Braking speed limit ---
        double omegaLimit = Math.sqrt(2.0 * aMax * Math.abs(error));
        omegaLimit = Math.min(omegaLimit, omegaMax);
        lastOmegaLimit = omegaLimit;

        // Position -> desired velocity, then cap it by braking limit
        double omegaCmd = kPos * error;                 // (deg/sec)
        omegaCmd = util.clamp(omegaCmd, -omegaLimit, omegaLimit);
        lastOmegaCmd = omegaCmd;

        // Velocity control (simple P) + static friction feedforward
        double velErr = omegaCmd - omegaFilt;
        lastVelErr = velErr;
        double power = kVel * velErr;

        // Apply kS mainly to break static friction; taper it out when already moving / near target.
        if (Math.abs(error) > posTol) {
            double kSDir;
            if (error > 0) {
                kSDir = (kSPos != 0.0) ? kSPos : kS;
            } else {
                kSDir = (kSNeg != 0.0) ? kSNeg : kS;
            }

            double errScale = util.clamp(Math.abs(error) / Math.max(1e-6, kSErrorFullDeg), 0.0, 1.0);
            double omegaScale = util.clamp(1.0 - (Math.abs(omegaFilt) / Math.max(1e-6, kSOmegaFullDegPerSec)), 0.0, 1.0);

            // Use desired motion direction (omegaCmd) rather than raw error sign.
            power += kSDir * sign(omegaCmd) * errScale * omegaScale;
        }

        // --- Integral hold (optional) ---
        // Purpose: remove residual steady-state error from stiction/bias without needing huge kS.
        // Only active near target and at low speed.
        if (kIHold != 0.0) {
            boolean inZone = (Math.abs(error) <= iZoneDeg) && (Math.abs(omegaFilt) <= iZoneOmegaDegPerSec);
            lastIInZone = inZone;

            if (inZone) {
                // Reset when error crosses through 0 to avoid "fighting" the new direction.
                if (sign(error) != sign(lastIError) && Math.abs(lastIError) > posTol) {
                    iPower = 0.0;
                }

                // Anti-windup: don't integrate further if we're already saturated.
                boolean saturated = Math.abs(power) >= (maxPower - 1e-3);
                lastISaturated = saturated;
                if (!saturated) {
                    iPower += kIHold * error * dt;
                    iPower = util.clamp(iPower, -iMaxPower, iMaxPower);
                }
            } else {
                lastISaturated = false;
                // Leak iPower toward 0 when not in the integrate zone.
                if (iLeakPerSec > 0.0) {
                    double leak = util.clamp(iLeakPerSec * dt, 0.0, 1.0);
                    iPower *= (1.0 - leak);
                }
            }

            lastIError = error;
            power += iPower;
        }

        // Clamp output
        power = util.clamp(power, -maxPower, maxPower);

        // Slew limit output power to prevent immediate high speed / overshoot
        if (powerSlewPerSec > 0.0 && dt > 1e-4) {
            double maxDelta = powerSlewPerSec * dt;
            power = util.clamp(power, lastOutPower - maxDelta, lastOutPower + maxDelta);
        }

        lastOutPower = power;
        spindexPower = power;
        return spindexPower;
    }

    public void setSpindex(double dt){
        setSpindexPow(getSpindexPID(currentPos, targetPosUnwrapped, dt));
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        timer.reset();

        // Initialize unwrap state cleanly
        lastVoltage = spindexEnc.getVoltage();
        double raw = spindexEnc.getCurrentPosition(); // 0..360
        lastRawDeg = raw;
        unwrappedPos = raw;
        lastRawPos = raw;
        hasRaw = true;

        currentPos = unwrappedPos;

        posFilt = currentPos;
        hasPosFilt = false;

        // Initialize velocity state cleanly
        lastPos = currentPos;
        hasLast = false;
        omegaFilt = 0.0;
        lastOutPower = 0.0;
        iPower = 0.0;
        lastIError = 0.0;
    }

    @Override
    public void update(){
        double dt = timer.seconds();
        timer.reset();

        lastDt = dt;

        // Allow live calibration changes via @Configurable (applies in both PID and manual modes)
        spindexEnc.zero(encOffsetDeg);
        spindexEnc.setInverted(encInverted);

        // Always update sensor state so telemetry/logging/characterization can see motion
        // even when PID is disabled.
        lastVoltage = spindexEnc.getVoltage();
        double raw = spindexEnc.getCurrentPosition();
        lastRawDeg = raw;
        currentPos = unwrapDegrees(raw);

        // Optional filtering on unwrapped position (helps noisy analog)
        double alpha = util.clamp(posFilterAlpha, 0.0, 1.0);
        if (!hasPosFilt) {
            posFilt = currentPos;
            hasPosFilt = true;
        } else {
            posFilt = posFilt + alpha * (currentPos - posFilt);
        }

        // Convert raw target (0..360) into nearest unwrapped target
        targetPosUnwrapped = nearestTargetUnwrapped(posFilt, spindexTarget);

        if (useSpindexPID){
            // PID path updates velocity internally (so it happens exactly once per loop).
            setSpindexPow(getSpindexPID(posFilt, targetPosUnwrapped, dt));
        } else {
            // Manual path still needs velocity estimates for logging/characterization.
            if (dt <= 1e-4) dt = 1e-4;
            double omega = 0.0;
            if (hasLast) omega = (posFilt - lastPos) / dt;
            lastPos = posFilt;
            hasLast = true;
            lastOmega = omega;
            omegaFilt = omegaFilt + velFilterAlpha * (omega - omegaFilt);

            setSpindexPow(util.clamp(manualPower, -1.0, 1.0));
        }
    }
}
