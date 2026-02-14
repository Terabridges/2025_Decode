package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="SpindexTuner", group="Test")
public class SpindexTuner extends OpMode {

    Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    // Tuning knobs
    public static double stepDeg = 60.0;
    public static double fineStepDeg = 5.0;
    public static double manualMaxPower = 0.2;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        // These show up in the log metadata and can help you filter runs in AdvantageScope.
        Logger.recordMetadata("OpMode", "SpindexTuner");
        Logger.recordMetadata("Robot", "2025_Decode");

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

    }

    @Override
    public void start(){
        //robot.toInit();
        robot.intake.spindex.toInit();
    }

    @Override
    public void loop() {
        gamepadUpdate();

        // --- Controls ---
        // A: toggle PID hold
        if (currentGamepad1.a && !previousGamepad1.a) {
            robot.intake.spindex.useSpindexPID = !robot.intake.spindex.useSpindexPID;
            if (robot.intake.spindex.useSpindexPID) {
                // When enabling PID, snap target to current raw angle so it holds immediately.
                Spindex.spindexTarget = wrap0To360(robot.intake.spindex.getLastRawDeg());
            }
        }

        // B: set target to current ("hold here")
        if (currentGamepad1.b && !previousGamepad1.b) {
            Spindex.spindexTarget = wrap0To360(robot.intake.spindex.getLastRawDeg());
            robot.intake.spindex.useSpindexPID = true;
        }

        // LB/RB: coarse step
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget - stepDeg);
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget + stepDeg);
        }

        // Dpad left/right: fine step
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget - fineStepDeg);
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            Spindex.spindexTarget = wrap0To360(Spindex.spindexTarget + fineStepDeg);
        }

        // Triggers: manual override power (temporarily disables PID)
        double manual = (currentGamepad1.right_trigger - currentGamepad1.left_trigger) * manualMaxPower;
        boolean manualActive = Math.abs(manual) > 0.02;
        if (manualActive) {
            robot.intake.spindex.useSpindexPID = false;
            Spindex.manualPower = manual;
        } else {
            Spindex.manualPower = 0.0;

            // If we just released manual, re-enable PID and hold current.
            boolean wasManual = (Math.abs(previousGamepad1.right_trigger - previousGamepad1.left_trigger) * manualMaxPower) > 0.02;
            if (wasManual) {
                Spindex.spindexTarget = wrap0To360(robot.intake.spindex.getLastRawDeg());
                robot.intake.spindex.useSpindexPID = true;
            }
        }

        // --- Update subsystem ---
        robot.intake.spindex.update();

        // --- Telemetry (PsiKitAutoLog will capture these too) ---
        joinedTelemetry.addData("Batt (V)", robot.getVoltage());
        joinedTelemetry.addData("PID Enabled", robot.intake.spindex.useSpindexPID);
        joinedTelemetry.addData("Manual Power", Spindex.manualPower);

        joinedTelemetry.addData("Enc Volts", robot.intake.spindex.getLastVoltage());
        joinedTelemetry.addData("Raw Deg", robot.intake.spindex.getLastRawDeg());
        joinedTelemetry.addData("Pos Unwrapped", robot.intake.spindex.getCurrentPosition());
        joinedTelemetry.addData("Pos Filtered", robot.intake.spindex.getCurrentPositionFiltered());

        joinedTelemetry.addData("Target Raw", robot.intake.spindex.getTargetPosition());
        joinedTelemetry.addData("Target Unwrapped", robot.intake.spindex.getTargetPositionUnwrapped());
        joinedTelemetry.addData("Err (deg)", robot.intake.spindex.getLastErrorDeg());

        joinedTelemetry.addData("Omega Filt (deg/s)", robot.intake.spindex.getLastOmegaDegPerSec());
        joinedTelemetry.addData("Omega Cmd (deg/s)", robot.intake.spindex.getLastOmegaCmdDegPerSec());
        joinedTelemetry.addData("Omega Limit (deg/s)", robot.intake.spindex.getLastOmegaLimitDegPerSec());
        joinedTelemetry.addData("Vel Err (deg/s)", robot.intake.spindex.getLastVelErrDegPerSec());

        joinedTelemetry.addData("Power", robot.intake.spindex.getLastPower());
        joinedTelemetry.addData("dt (s)", robot.intake.spindex.getLastDt());

        joinedTelemetry.addData("stepDeg", stepDeg);
        joinedTelemetry.addData("fineStepDeg", fineStepDeg);
        joinedTelemetry.update();

        // --- PsiKit outputs (these are what become live keys + rlog data) ---
        Logger.recordOutput("Spindex/BattV", robot.getVoltage());
        Logger.recordOutput("Spindex/PidEnabled", robot.intake.spindex.useSpindexPID);
        Logger.recordOutput("Spindex/ManualPower", Spindex.manualPower);

        Logger.recordOutput("Spindex/EncVolts", robot.intake.spindex.getLastVoltage());
        Logger.recordOutput("Spindex/RawDeg", robot.intake.spindex.getLastRawDeg());
        Logger.recordOutput("Spindex/PosUnwrappedDeg", robot.intake.spindex.getCurrentPosition());
        Logger.recordOutput("Spindex/PosFilteredDeg", robot.intake.spindex.getCurrentPositionFiltered());

        Logger.recordOutput("Spindex/TargetRawDeg", robot.intake.spindex.getTargetPosition());
        Logger.recordOutput("Spindex/TargetUnwrappedDeg", robot.intake.spindex.getTargetPositionUnwrapped());
        Logger.recordOutput("Spindex/ErrorDeg", robot.intake.spindex.getLastErrorDeg());

        Logger.recordOutput("Spindex/OmegaFiltDegPerSec", robot.intake.spindex.getLastOmegaDegPerSec());
        Logger.recordOutput("Spindex/OmegaCmdDegPerSec", robot.intake.spindex.getLastOmegaCmdDegPerSec());
        Logger.recordOutput("Spindex/OmegaLimitDegPerSec", robot.intake.spindex.getLastOmegaLimitDegPerSec());
        Logger.recordOutput("Spindex/VelErrDegPerSec", robot.intake.spindex.getLastVelErrDegPerSec());

        Logger.recordOutput("Spindex/Power", robot.intake.spindex.getLastPower());
        Logger.recordOutput("Spindex/DtSec", robot.intake.spindex.getLastDt());

        Logger.recordOutput("Spindex/StepDeg", stepDeg);
        Logger.recordOutput("Spindex/FineStepDeg", fineStepDeg);

    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    private static double wrap0To360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }
}
