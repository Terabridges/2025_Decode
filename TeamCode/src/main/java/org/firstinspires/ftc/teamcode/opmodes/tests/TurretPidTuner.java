package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@Configurable
@TeleOp(name = "TurretPidTuner", group = "Test")
public class TurretPidTuner extends LinearOpMode {

    // Panels-editable PID and limits (LockTester style)
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double targetDeg = 0.0;
    public static double maxPow = 0.4;
    public static double deadband = 0.0;

    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    private PIDController pid;

    @Override
    public void runOpMode() {

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        // Vision unused here; only turret encoder + PID
        Shooter shooter = new Shooter(hardwareMap, (Vision) null);

        pid = new PIDController(p, i, d);

        telemetry.addLine("Turret PID Tuner (Panels editable)");
        telemetry.addLine("LB/RB manual power; A resets PID; Y zero target");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);

            if (current.a && !previous.a) {
                pid.reset();
            }
            if (current.y && !previous.y) {
                targetDeg = 0.0;
            }

            // Manual bump
            double manualPow = 0.0;
            if (current.left_bumper) manualPow = 0.3;
            else if (current.right_bumper) manualPow = -0.3;

            pid.setPID(p, i, d);
            double error = wrapDeg(targetDeg - shooter.getTurretPos());
            if (Math.abs(error) < deadband) error = 0.0;
            double cmd = pid.calculate(0.0, error); // drive error to zero
            cmd = clamp(cmd, -maxPow, maxPow);

            if (manualPow != 0.0) {
                shooter.setTurretPower(manualPow);
            } else {
                shooter.setTurretPower(cmd);
            }

            joinedTelemetry.addData("TargetDeg", "%.2f", targetDeg);
            joinedTelemetry.addData("TurretPos", "%.2f", shooter.getTurretPos());
            joinedTelemetry.addData("Error", "%.2f", error);
            joinedTelemetry.addData("P/I/D", "%.5f / %.5f / %.5f", p, i, d);
            joinedTelemetry.addData("Cmd Pow", cmd);
            joinedTelemetry.addData("ManualPow", manualPow);
            joinedTelemetry.update();
        }
    }

    private double wrapDeg(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
