package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretAimController;

@Configurable
@TeleOp(name = "TurretEncoderAccuracyTest", group = "Test")
public class TurretEncoderAccuracyTest extends OpMode {

    public static double stepDeg = 5.0;
    public static double settleToleranceDeg = 2.0;
    public static double settleTimeMs = 300.0;

    public static double presetA = TurretAimController.turretForwardDeg;
    public static double presetX = 90.0;
    public static double presetY = 180.0;
    public static double presetB = 270.0;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    private final ElapsedTime inToleranceTimer = new ElapsedTime();

    private double targetDeg = TurretAimController.turretForwardDeg;
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
            targetDeg = robot.outtake.turret.getCurrentDegrees();
        }

        robot.outtake.turret.setAimLockEnabled(false);
        robot.outtake.turret.setTargetAngle(targetDeg);
        robot.outtake.turret.update();

        double encoderDeg = robot.outtake.turret.getEncoderRawDeg();
        double mappedDeg = robot.outtake.turret.getCurrentDegrees();
        double commandedDeg = mappedDeg;
        double errorDeg = wrapSignedDeg(mappedDeg - targetDeg);
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
        joinedTelemetry.addData("Encoder Raw (deg)", "%.2f", encoderDeg);
        joinedTelemetry.addData("Mapped Position (deg)", "%.2f", mappedDeg);
        joinedTelemetry.addData("Error Mapped-Target (deg)", "%.2f", errorDeg);
        joinedTelemetry.addData("Abs Error (deg)", "%.2f", absErrorDeg);
        joinedTelemetry.addData("On Target", robot.outtake.turret.isOnTarget());
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
