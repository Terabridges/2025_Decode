package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;

@Configurable
@TeleOp(name = "TurretEncoderAccuracyTest", group = "Test")
public class TurretEncoderAccuracyTest extends OpMode {

    public static double stepDeg = 5.0;
    public static double settleToleranceDeg = 2.0;
    public static double settleTimeMs = 300.0;

    public static double presetA = Turret.turretForwardDeg;
    public static double presetX = 90.0;
    public static double presetY = 180.0;
    public static double presetB = 270.0;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    private final ElapsedTime inToleranceTimer = new ElapsedTime();

    private double targetDeg = Turret.turretForwardDeg;
    private boolean wasInTolerance = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
    }

    @Override
    public void start() {
        robot.toInit();
        robot.outtake.turret.setAimLockEnabled(false);
        Turret.turretVelocity = 0.0;
        targetDeg = robot.outtake.turret.getCurrentDegrees();
        inToleranceTimer.reset();
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);

        if (edge(current.a, previous.a)) targetDeg = presetA;
        if (edge(current.x, previous.x)) targetDeg = presetX;
        if (edge(current.y, previous.y)) targetDeg = presetY;
        if (edge(current.b, previous.b)) targetDeg = presetB;

        if (edge(current.dpad_right, previous.dpad_right)) targetDeg += stepDeg;
        if (edge(current.dpad_left, previous.dpad_left)) targetDeg -= stepDeg;

        if (edge(current.back, previous.back)) {
            robot.outtake.turret.toInit();
            Turret.turretVelocity = 0.0;
            targetDeg = robot.outtake.turret.getCurrentDegrees();
        }

        robot.outtake.turret.setAimLockEnabled(false);
        Turret.turretVelocity = 0.0;
        robot.outtake.turret.setTurretDegree(targetDeg);
        robot.outtake.turret.update();

        double encoderDeg = robot.outtake.turret.getEncoderDegrees();
        double mappedEncoderTurretDeg = robot.outtake.turret.getMappedEncoderTurretDegrees();
        double commandedDeg = robot.outtake.turret.getCurrentDegrees();
        double errorDeg = wrapSignedDeg(encoderDeg - targetDeg);
        double mappedErrorDeg = robot.outtake.turret.getMappedEncoderErrorDeg(targetDeg);
        double absErrorDeg = Math.abs(errorDeg);

        boolean inTolerance = absErrorDeg <= Math.abs(settleToleranceDeg);
        if (inTolerance) {
            if (!wasInTolerance) {
                inToleranceTimer.reset();
            }
        } else {
            inToleranceTimer.reset();
        }
        wasInTolerance = inTolerance;
        boolean settled = inTolerance && inToleranceTimer.milliseconds() >= settleTimeMs;

        joinedTelemetry.addData("Target (deg)", "%.2f", targetDeg);
        joinedTelemetry.addData("Commanded (deg)", "%.2f", commandedDeg);
        joinedTelemetry.addData("Encoder (deg)", "%.2f", encoderDeg);
        joinedTelemetry.addData("Error Enc-Target (deg)", "%.2f", errorDeg);
        joinedTelemetry.addData("Mapped Enc->Turret (deg)", "%.2f", mappedEncoderTurretDeg);
        joinedTelemetry.addData("Mapped Error (deg)", "%.2f", mappedErrorDeg);
        joinedTelemetry.addData("Abs Error (deg)", "%.2f", absErrorDeg);
        joinedTelemetry.addData("Turret Velocity Cmd", "%.2f", Turret.turretVelocity);
        joinedTelemetry.addData("Tolerance (deg)", "%.2f", settleToleranceDeg);
        joinedTelemetry.addData("In Tolerance", inTolerance);
        joinedTelemetry.addData("Settled", settled);
        joinedTelemetry.addData("InTol Time (ms)", "%.1f", inToleranceTimer.milliseconds());
        joinedTelemetry.addData("Controls", "A/X/Y/B presets, Dpad L/R step, BACK reset");
        joinedTelemetry.update();
    }

    private boolean edge(boolean now, boolean prevVal) {
        return now && !prevVal;
    }

    private double wrapSignedDeg(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
