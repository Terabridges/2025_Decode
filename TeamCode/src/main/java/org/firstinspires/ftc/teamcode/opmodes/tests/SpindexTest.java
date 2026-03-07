package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="SpindexTest", group="Test")
public class SpindexTest extends OpMode {

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    private JoinedTelemetry joinedTelemetry;
    private Spindex spindex;
    private AnalogInput floodgateAnalog;

    public static double commandDegree = 0;
    public static double dpadFineStepDeg = 5.0;
    public static double dpadStepDeg = 60.0;
    public static double bumperStepDeg = 120.0;
    public static boolean holdCommand = true;

    private double lastAppliedDegree = Double.NaN;
    private double floodgateCurrent = Double.NaN;

    @Override
    public void init() {
        spindex = new Spindex(hardwareMap);
        floodgateAnalog = hardwareMap.get(AnalogInput.class, "floodgate");
        floodgateCurrent = floodgateAnalog.getVoltage() / 3.3 * 80.0;

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
    }

    @Override
    public void start(){
        commandDegree = wrapEncoderDeg(spindex.getAbsolutePos());
        applyCommand();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        floodgateCurrent = floodgateAnalog.getVoltage() / 3.3 * 80.0;
        if (edge(currentGamepad1.dpad_up, previousGamepad1.dpad_up)) {
            commandDegree = wrapEncoderDeg(commandDegree + dpadStepDeg);
        }
        if (edge(currentGamepad1.dpad_down, previousGamepad1.dpad_down)) {
            commandDegree = wrapEncoderDeg(commandDegree - dpadStepDeg);
        }
        if (edge(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) {
            commandDegree = wrapEncoderDeg(commandDegree + dpadFineStepDeg);
        }
        if (edge(currentGamepad1.dpad_left, previousGamepad1.dpad_left)) {
            commandDegree = wrapEncoderDeg(commandDegree - dpadFineStepDeg);
        }
        if (edge(currentGamepad1.right_bumper, previousGamepad1.right_bumper)) {
            commandDegree = wrapEncoderDeg(commandDegree + bumperStepDeg);
        }
        if (edge(currentGamepad1.left_bumper, previousGamepad1.left_bumper)) {
            commandDegree = wrapEncoderDeg(commandDegree - bumperStepDeg);
        }

        if (edge(currentGamepad1.x, previousGamepad1.x)) {
            holdCommand = !holdCommand;
        }

        if (edge(currentGamepad1.a, previousGamepad1.a)) {
            applyCommand();
        }

        if (holdCommand) {
            applyCommand();
        }

        double absoluteDeg = spindex.getAbsolutePos();
        double commandedDeg = spindex.getCommandedPos();
        double errorDeg = wrapSignedEncoderDeg(commandedDeg - absoluteDeg);

        Logger.recordOutput("SpindexTest/FloodgateAmps", floodgateCurrent);
        Logger.recordOutput("SpindexTest/CommandDeg", commandDegree);
        Logger.recordOutput("SpindexTest/AbsoluteDeg", absoluteDeg);
        Logger.recordOutput("SpindexTest/CommandedDeg", commandedDeg);
        Logger.recordOutput("SpindexTest/ErrorDeg", errorDeg);
        Logger.recordOutput("FloodgateAmps", floodgateCurrent);

        joinedTelemetry.addData("Command Degree (encoder frame)", "%.2f", commandDegree);
        joinedTelemetry.addData("Last Applied Degree", "%.2f", lastAppliedDegree);
        joinedTelemetry.addData("Absolute Encoder Degree", "%.2f", absoluteDeg);
        joinedTelemetry.addData("Commanded Degree (from servo pos)", "%.2f", commandedDeg);
        joinedTelemetry.addData("Command Error (deg)", "%.2f", errorDeg);
        joinedTelemetry.addData("Floodgate Current (A)", "%.2f", floodgateCurrent);
        joinedTelemetry.addData("Abs Encoder Ratio", "%.3f", Spindex.absoluteEncoderGearRatio);
        joinedTelemetry.addData("Command Ratio", "%.3f", Spindex.commandGearRatio);
        joinedTelemetry.addData("Command Bias (deg)", "%.2f", Spindex.commandBiasDeg);
        joinedTelemetry.addData("Hold Command", holdCommand);
        joinedTelemetry.addData("Controls", "Dpad Up/Down: ±60, Left/Right: fine, LB/RB: ±120, A apply once, X toggle hold");
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
    }

    private void applyCommand() {
        commandDegree = wrapEncoderDeg(commandDegree);
        spindex.setSpindexDegree(commandDegree);
        lastAppliedDegree = commandDegree;
    }

    private static boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private static double encoderRangeDeg() {
        return 360.0 * Math.max(1e-6, Math.abs(Spindex.absoluteEncoderGearRatio));
    }

    private static double wrapEncoderDeg(double deg) {
        double range = encoderRangeDeg();
        return ((deg % range) + range) % range;
    }

    private static double wrapSignedEncoderDeg(double deg) {
        double range = encoderRangeDeg();
        double wrapped = ((deg + range * 0.5) % range + range) % range - range * 0.5;
        return wrapped;
    }
}