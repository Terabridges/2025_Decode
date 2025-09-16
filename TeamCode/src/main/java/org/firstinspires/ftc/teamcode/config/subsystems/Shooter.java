package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter implements Subsystem {

    /** Config names (override from Robot if you rename in RC config) */
    public String leftName = "shooterL";
    public String rightName = "shooterR";
    public String hoodName = "hood";

    /** Hardware */
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public Servo hood;

    /** Cached context */
    private HardwareMap hw;
    private Telemetry tele;

    /** State */
    private double desiredRpm = 0.0;
    private boolean isRunning = false;
    private double hoodPos = 0.5;       // 0..1
    private int directionMultiplier = 1; // +1 forward, -1 reverse

    /** Which side has the encoder? If false, right has the encoder. */
    public boolean encoderOnLeft = true;

    /** RPM estimation */
    private final ElapsedTime rpmTimer  = new ElapsedTime();
    private int lastTicks = 0;

    // Tune these for your motors/gearbox
    private static final double TICKS_PER_REV    = 28.0; // goBILDA 5202/5203
    private static final double GEAR_RATIO       = 1.0;   // motor->wheel ( >1 if geared down )

    // ---------------- Lifecycle ----------------

    public Shooter(HardwareMap hw, Telemetry tele) {
        this.hw = hw;
        this.tele = tele;
    }

    @Override
    public void toInit() {
        leftMotor  = hw.get(DcMotorEx.class, leftName);
        rightMotor = hw.get(DcMotorEx.class, rightName);
        hood  = hw.get(Servo.class,      hoodName);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Encoder configuration: encoder side uses RUN_USING_ENCODER, the other can run open-loop
        if (encoderOnLeft) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        setHoodPos(hoodPos);
        setRpm(0);

        rpmTimer.reset();
        lastTicks = encoderOnLeft ? leftMotor.getCurrentPosition() : rightMotor.getCurrentPosition();
    }

    @Override
    public void update() {
        // Clamp desired RPM
        if (desiredRpm < 0) desiredRpm = 0;
        if (desiredRpm > 6000) desiredRpm = 6000;

        // Convert RPM to ticks/sec
        double velocity = (desiredRpm * TICKS_PER_REV) / 60.0;
        if (!isRunning) {
            velocity = 0;
        }

        // Apply velocity with direction multiplier
        if (leftMotor != null)  leftMotor.setVelocity(velocity * directionMultiplier);
        if (rightMotor != null) rightMotor.setVelocity(velocity * directionMultiplier);

        // Telemetry (including est. RPM)
        if (tele != null) {
            double dtRpm = Math.max(1e-3, rpmTimer.seconds());
            rpmTimer.reset();
            int cur = encoderOnLeft ? leftMotor.getCurrentPosition() : rightMotor.getCurrentPosition();
            int d = cur - lastTicks;
            lastTicks = cur;

            double tps = d / dtRpm; // ticks/sec
            double rps = (tps / TICKS_PER_REV) / GEAR_RATIO;
            double rpm = rps * 60.0;

            tele.addData("Shooter tgtRPM", "%.0f", desiredRpm);
            tele.addData("Running", isRunning);
            tele.addData("EncSide", encoderOnLeft ? "LEFT" : "RIGHT");
            tele.addData("Direction", directionMultiplier == 1 ? "FWD" : "REV");
            tele.addData("HoodPos", "%.2f", hoodPos);
            tele.addData("RPM(est)", "%.0f", rpm);
        }
    }

    // ---------------- Methods ----------------

    /** Set target RPM. */
    public void setRpm(double rpm) {
        desiredRpm = rpm;
        isRunning = rpm > 0;
    }

    /** Reverse the shooter direction (multiplies power by -1). */
    public void reverseDirection() {
        directionMultiplier *= -1;
    }

    public void stop() { setRpm(0); }

    /** True if flagged running. */
    public boolean isSpinning() { return isRunning; }

    /** Toggle convenience for testing. */
    public void toggle(double rpm) { if (isSpinning()) stop(); else setRpm(rpm); }

    /** Hood helpers */
    public void setHoodPos(double pos) {
        hoodPos = Range.clip(pos, 0.0, 1.0);
        if (hood != null) hood.setPosition(hoodPos);
    }
    public void nudgeHood(double delta) { setHoodPos(hoodPos + delta); }

    /** RPM nudge useful during on-field tuning. */
    public void nudgeRpm(double delta) { setRpm(desiredRpm + delta); }

    // Accessors for logging or UI
    public double getTargetRpm()  { return desiredRpm; }
    public double getHood()       { return hoodPos; }
}
