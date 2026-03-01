package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretController;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretHardware;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

/**
 * Trapezoidal motion profile tuner for the CRServo turret controller.
 *
 * <h3>Purpose</h3>
 * Sends the turret to preset target angles using the full PID+FF+Profile pipeline,
 * logging profile setpoint vs actual position for each move. Allows tuning of
 * motion profile constraints (max velocity, max acceleration) and PID gains in
 * real-time via the dashboard.
 *
 * <h3>Metrics Displayed</h3>
 * <ul>
 *   <li>Tracking error (setpoint − actual) during the move</li>
 *   <li>Overshoot (peak error after reaching target zone)</li>
 *   <li>Settle time (time from profile end to on-target)</li>
 *   <li>Total move time</li>
 *   <li>Peak velocity achieved</li>
 * </ul>
 *
 * <h3>Controls</h3>
 * <ul>
 *   <li><b>A</b> — go to Target A</li>
 *   <li><b>B</b> — go to Target B</li>
 *   <li><b>X</b> — go to Target C</li>
 *   <li><b>Y</b> — go to panel-configurable target</li>
 *   <li><b>Dpad Up</b> — go to center (180°)</li>
 *   <li><b>Left Stick X</b> — manual nudge (when no profile active)</li>
 *   <li><b>BACK</b> — stop and hold current position</li>
 * </ul>
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretProfileTuner", group = "Test")
public class TurretProfileTuner extends OpMode {
    private static final String LOG_PREFIX = "Calibrator/Profile/";

    //---------------- Configurable Target Angles ----------------
    public static double targetA = 100.0;
    public static double targetB = 220.0;
    public static double targetC = 80.0;
    public static double panelTarget = 163.0;

    //---------------- Manual ----------------
    public static double nudgePowerScale = 0.15;
    public static double nudgeDeadband = 0.05;

    //---------------- State ----------------
    private TurretHardware hardware;
    private TurretController controller;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    // Move metrics
    private boolean moveActive = false;
    private double moveStartDeg = 0.0;
    private double moveGoalDeg = 0.0;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private double peakTrackingError = 0.0;
    private double overshoot = 0.0;
    private double settleTimeSec = Double.NaN;
    private double peakVelocity = 0.0;
    private double totalMoveTimeSec = Double.NaN;
    private boolean profileFinished = false;
    private double profileEndTimeSec = Double.NaN;

    @Override
    public void init() {
        hardware = new TurretHardware(hardwareMap);
        controller = new TurretController();
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
    }

    @Override
    public void start() {
        hardware.update();
        controller.reset();
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);
        hardware.update();

        double posDeg = hardware.getPositionDeg();
        double velDegSec = hardware.getVelocityDegPerSec();

        // --- Input handling ---
        if (edge(current.a, previous.a)) startMove(posDeg, targetA);
        if (edge(current.b, previous.b)) startMove(posDeg, targetB);
        if (edge(current.x, previous.x)) startMove(posDeg, targetC);
        if (edge(current.y, previous.y)) startMove(posDeg, panelTarget);
        if (edge(current.dpad_up, previous.dpad_up)) startMove(posDeg, 180.0);

        // Stop & hold
        if (edge(current.back, previous.back)) {
            controller.setTargetAngleImmediate(posDeg);
            moveActive = false;
        }

        // Manual nudge when no profile active
        if (controller.getMode() != TurretController.ControlMode.PROFILE) {
            double nudge = current.left_stick_x;
            if (Math.abs(nudge) < nudgeDeadband) nudge = 0.0;
            if (Math.abs(nudge) > 0.0) {
                controller.setDirectPower(nudge * nudgePowerScale);
            }
        }

        // --- Controller update ---
        double power = controller.update(posDeg, velDegSec);
        hardware.setPower(power);

        // --- Track move metrics ---
        if (moveActive) {
            updateMoveMetrics(posDeg, velDegSec);
        }

        // --- Logging ---
        Logger.recordOutput(LOG_PREFIX + "PositionDeg", posDeg);
        Logger.recordOutput(LOG_PREFIX + "VelocityDegSec", velDegSec);
        Logger.recordOutput(LOG_PREFIX + "Power", power);
        Logger.recordOutput(LOG_PREFIX + "ControlMode", controller.getMode());
        Logger.recordOutput(LOG_PREFIX + "TargetDeg", controller.getTargetDeg());
        Logger.recordOutput(LOG_PREFIX + "MoveActive", moveActive);
        Logger.recordOutput(LOG_PREFIX + "PeakTrackingError", peakTrackingError);
        Logger.recordOutput(LOG_PREFIX + "Overshoot", overshoot);
        Logger.recordOutput(LOG_PREFIX + "PeakVelocity", peakVelocity);

        if (!Double.isNaN(settleTimeSec)) {
            Logger.recordOutput(LOG_PREFIX + "SettleTimeSec", settleTimeSec);
        }
        if (!Double.isNaN(totalMoveTimeSec)) {
            Logger.recordOutput(LOG_PREFIX + "TotalMoveTimeSec", totalMoveTimeSec);
        }

        // --- Telemetry ---
        joinedTelemetry.addData("Position Deg", fmt(posDeg));
        joinedTelemetry.addData("Velocity Deg/s", fmt(velDegSec));
        joinedTelemetry.addData("Power", fmt(power));
        joinedTelemetry.addData("Control Mode", controller.getMode());
        joinedTelemetry.addData("Target Deg", fmt(controller.getTargetDeg()));
        joinedTelemetry.addData("On Target", controller.isOnTarget(posDeg, velDegSec));

        joinedTelemetry.addLine("--- Last Move Metrics ---");
        joinedTelemetry.addData("Move Active", moveActive);
        joinedTelemetry.addData("Start → Goal", fmt(moveStartDeg) + "° → " + fmt(moveGoalDeg) + "°");
        joinedTelemetry.addData("Peak Tracking Error", fmt(peakTrackingError) + "°");
        joinedTelemetry.addData("Overshoot", fmt(overshoot) + "°");
        joinedTelemetry.addData("Peak Velocity", fmt(peakVelocity) + "°/s");
        joinedTelemetry.addData("Settle Time", fmtTime(settleTimeSec));
        joinedTelemetry.addData("Total Move Time", fmtTime(totalMoveTimeSec));
        joinedTelemetry.addData("Profile Finished", profileFinished);

        if (moveActive) {
            double elapsed = moveTimer.seconds();
            joinedTelemetry.addData("Move Elapsed", fmt(elapsed) + "s");
            double profileTotal = controller.getProfile().getTotalTime();
            joinedTelemetry.addData("Profile Duration", fmt(profileTotal) + "s");
        }

        joinedTelemetry.addLine("--- Profile Constraints ---");
        joinedTelemetry.addData("Max Velocity", fmt(TurretController.maxProfileVelocity) + "°/s");
        joinedTelemetry.addData("Max Acceleration", fmt(TurretController.maxProfileAcceleration) + "°/s²");

        joinedTelemetry.addLine("--- PID Gains ---");
        joinedTelemetry.addData("kP", fmt(TurretController.kP));
        joinedTelemetry.addData("kI", fmt(TurretController.kI));
        joinedTelemetry.addData("kD", fmt(TurretController.kD));
        joinedTelemetry.addData("kS_CW", fmt(TurretController.kS_CW));
        joinedTelemetry.addData("kS_CCW", fmt(TurretController.kS_CCW));
        joinedTelemetry.addData("kV", fmt(TurretController.kV));

        joinedTelemetry.addLine("--- Controls ---");
        joinedTelemetry.addLine("A/B/X: preset targets | Y: panel target | DUp: 180°");
        joinedTelemetry.addLine("BACK: stop | LStick X: nudge");
        joinedTelemetry.update();
    }

    private void startMove(double currentDeg, double goalDeg) {
        // Clamp goal to safe operating range
        goalDeg = TurretHardware.clampToSafeRange(goalDeg);

        moveStartDeg = currentDeg;
        moveGoalDeg = goalDeg;
        moveActive = true;
        profileFinished = false;
        peakTrackingError = 0.0;
        overshoot = 0.0;
        settleTimeSec = Double.NaN;
        totalMoveTimeSec = Double.NaN;
        profileEndTimeSec = Double.NaN;
        peakVelocity = 0.0;
        moveTimer.reset();

        controller.setTargetAngle(currentDeg, goalDeg);

        Logger.recordOutput(LOG_PREFIX + "Move/StartDeg", currentDeg);
        Logger.recordOutput(LOG_PREFIX + "Move/GoalDeg", goalDeg);
        Logger.recordOutput(LOG_PREFIX + "Move/Distance", wrapSigned(goalDeg - currentDeg));
    }

    private void updateMoveMetrics(double posDeg, double velDegSec) {
        double elapsed = moveTimer.seconds();
        double errorToGoal = wrapSigned(moveGoalDeg - posDeg);

        // Peak tracking error (during profile phase)
        if (controller.getMode() == TurretController.ControlMode.PROFILE) {
            // Tracking error = profile setpoint - actual
            // The controller internally tracks this, but we measure goal error
            peakTrackingError = Math.max(peakTrackingError, Math.abs(errorToGoal));
        }

        // Peak velocity
        peakVelocity = Math.max(peakVelocity, Math.abs(velDegSec));

        // Detect profile finished
        if (!profileFinished && controller.getMode() != TurretController.ControlMode.PROFILE) {
            profileFinished = true;
            profileEndTimeSec = elapsed;
        }

        // Overshoot: error sign change after crossing the goal
        if (profileFinished) {
            // Distance past the goal (overshoot is positive going past)
            double dirSign = Math.signum(wrapSigned(moveGoalDeg - moveStartDeg));
            double pastGoal = -errorToGoal * dirSign; // positive if past goal
            if (pastGoal > overshoot) {
                overshoot = pastGoal;
            }
        }

        // Settle time
        if (Double.isNaN(settleTimeSec) && profileFinished
                && controller.isOnTarget(posDeg, velDegSec)) {
            settleTimeSec = elapsed - (Double.isNaN(profileEndTimeSec) ? 0 : profileEndTimeSec);
            totalMoveTimeSec = elapsed;
            moveActive = false; // Move complete

            Logger.recordOutput(LOG_PREFIX + "Move/Complete", true);
            Logger.recordOutput(LOG_PREFIX + "Move/TotalTimeSec", totalMoveTimeSec);
            Logger.recordOutput(LOG_PREFIX + "Move/SettleTimeSec", settleTimeSec);
            Logger.recordOutput(LOG_PREFIX + "Move/Overshoot", overshoot);
            Logger.recordOutput(LOG_PREFIX + "Move/PeakVelocity", peakVelocity);
            Logger.recordOutput(LOG_PREFIX + "Move/PeakTrackingError", peakTrackingError);
        }

        // Safety: if move takes too long, give up tracking
        if (elapsed > 10.0) {
            moveActive = false;
        }
    }

    private double wrapSigned(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private String fmt(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "---";
        return String.format("%.3f", v);
    }

    private String fmtTime(double sec) {
        if (Double.isNaN(sec)) return "---";
        return String.format("%.3f s", sec);
    }
}
