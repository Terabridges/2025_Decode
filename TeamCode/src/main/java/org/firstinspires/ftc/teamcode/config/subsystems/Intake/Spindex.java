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

    public boolean useSpindexPID = true;
    private double frontBallOne = 30;
    private double ballForward = 60;
    private double switchIntakeForward = 35;

    public static double kPos = 0.012;      // position -> velocity gain (1/sec)
    public static double kVel = 0.00035;    // velocity P (power per (deg/sec))
    public static double kS   = 0.08;       // static friction feedforward
    public static double maxPower = 0.75;

    public static double aMax = 6000;       // deg/sec^2  (tune)
    public static double omegaMax = 9000;   // deg/sec    (tune)
    public static double posTol = 5;        // deg
    public static double velTol = 50;       // deg/sec

    // State for velocity estimate
    private double lastPos = 0.0;      // UNWRAPPED pos last loop
    private boolean hasLast = false;
    private double omegaFilt = 0.0;
    public static double velFilterAlpha = 0.25; // 0..1 (higher = less filtering)

    // Unwrap state for absolute encoder (0..360)
    private double unwrappedPos = 0.0;
    private double lastRawPos = 0.0;
    private boolean hasRaw = false;
    private static final double WRAP_SIZE_DEG = 360.0;

    // For debug/telemetry if you want it later
    private double targetPosUnwrapped = 0.0;

    ElapsedTime timer;

    //---------------- Constructor ----------------
    public Spindex(HardwareMap map) {
        spindexLeft = map.get(CRServo.class, "spindexL");
        spindexRight = map.get(CRServo.class, "spindexR");
        spindexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontColor = map.get(RevColorSensorV3.class, "color1");
        middleColor = map.get(RevColorSensorV3.class, "color2");
        backColor = map.get(RevColorSensorV3.class, "color3");
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 0, 1);

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

    public double getTargetPosition(){
        return spindexTarget; // raw 0..360 as you had
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

        omegaFilt = omegaFilt + velFilterAlpha * (omega - omegaFilt);

        double error = target - pos;

        // Stop condition (prevents tiny oscillation near target)
        if (Math.abs(error) <= posTol && Math.abs(omegaFilt) <= velTol) {
            spindexPower = 0.0;
            return 0.0;
        }

        // --- Braking speed limit ---
        double omegaLimit = Math.sqrt(2.0 * aMax * Math.abs(error));
        omegaLimit = Math.min(omegaLimit, omegaMax);

        // Position -> desired velocity, then cap it by braking limit
        double omegaCmd = kPos * error;                 // (deg/sec)
        omegaCmd = util.clamp(omegaCmd, -omegaLimit, omegaLimit);

        // Velocity control (simple P) + static friction feedforward
        double velErr = omegaCmd - omegaFilt;
        double power = kVel * velErr;

        // Apply kS ONLY when we actually need to move (error-based gate)
        if (Math.abs(error) > posTol) {
            power += kS * sign(error);
        }

        // Clamp output
        power = util.clamp(power, -maxPower, maxPower);
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
        double raw = spindexEnc.getCurrentPosition(); // 0..360
        unwrappedPos = raw;
        lastRawPos = raw;
        hasRaw = true;

        currentPos = unwrappedPos;

        // Initialize velocity state cleanly
        lastPos = currentPos;
        hasLast = false;
        omegaFilt = 0.0;
    }

    @Override
    public void update(){
        double dt = timer.seconds();
        timer.reset();

        if (useSpindexPID){
            // Read raw 0..360, unwrap into continuous degrees
            double raw = spindexEnc.getCurrentPosition();
            currentPos = unwrapDegrees(raw);

            // Convert raw target (0..360) into nearest unwrapped target
            targetPosUnwrapped = nearestTargetUnwrapped(currentPos, spindexTarget);

            setSpindex(dt);
        } else {
            setSpindexPow(0);
        }
    }
}
