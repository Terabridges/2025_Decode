package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

@TeleOp(name = "Test: Color+Distance (PsiKit)", group = "Test")
public class ColorDistanceSensorPsiKitTest extends PsiKitIterativeOpMode {

    private static final int RLOG_PORT = 5802;
    private static final String SENSOR_NAME = "color_sensor";

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private ColorSensor rawColorSensor;
    private SwitchableLight switchableLight;
    private String acquisitionPath = "(not initialized)";

    private float gain = 2.0f;
    private boolean ledEnabled = true;
    private boolean lastDpadUp;
    private boolean lastDpadDown;
    private boolean lastA;

    @Override
    protected int getRlogPort() {
        return RLOG_PORT;
    }

    @Override
    protected void onPsiKitInit() {
        // Match Transfer: the robot is committed to REV Color Sensor V3.
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, SENSOR_NAME);
        acquisitionPath = "hardwareMap.get(RevColorSensorV3)";
        colorSensor = sensor;
        distanceSensor = sensor;
        rawColorSensor = sensor;
        switchableLight = (sensor instanceof SwitchableLight) ? (SwitchableLight) sensor : null;

        // Defaults that usually make color channels more meaningful.
        try {
            colorSensor.setGain(gain);
        } catch (Throwable ignored) {
            // Some proxy/replay setups may not support gain.
        }
        if (switchableLight != null) {
            try {
                switchableLight.enableLight(ledEnabled);
            } catch (Throwable ignored) {
                // Best-effort.
            }
        }

        telemetry.addLine("PsiKit logging active (OpMode).");
        telemetry.addData("Sensor name", SENSOR_NAME);
        telemetry.addData("Acquired via", acquisitionPath);
        telemetry.addData("Software LED supported", switchableLight != null);
        if (switchableLight == null) {
            telemetry.addLine("Note: REV color sensors typically use a physical LED switch.");
        }
        telemetry.update();
    }

    @Override
    protected void onPsiKitLoop() {
        // Simple controls:
        // - dpad up/down: increase/decrease gain
        // - A: toggle LED
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean a = gamepad1.a;

        if (dpadUp && !lastDpadUp) {
            gain = Math.min(50.0f, gain + 1.0f);
            try { colorSensor.setGain(gain); } catch (Throwable ignored) { }
        }
        if (dpadDown && !lastDpadDown) {
            gain = Math.max(1.0f, gain - 1.0f);
            try { colorSensor.setGain(gain); } catch (Throwable ignored) { }
        }
        if (a && !lastA) {
            ledEnabled = !ledEnabled;
            if (switchableLight != null) {
                try { switchableLight.enableLight(ledEnabled); } catch (Throwable ignored) { }
            }
        }

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastA = a;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distanceIn = distanceSensor.getDistance(DistanceUnit.INCH);
        double distanceM = distanceSensor.getDistance(DistanceUnit.METER);

        int rawR = (rawColorSensor != null) ? rawColorSensor.red() : -1;
        int rawG = (rawColorSensor != null) ? rawColorSensor.green() : -1;
        int rawB = (rawColorSensor != null) ? rawColorSensor.blue() : -1;
        int rawA = (rawColorSensor != null) ? rawColorSensor.alpha() : -1;

        double normSum = colors.red + colors.green + colors.blue;
        double normR = (normSum > 1e-6) ? (colors.red / normSum) : 0.0;
        double normG = (normSum > 1e-6) ? (colors.green / normSum) : 0.0;
        double normB = (normSum > 1e-6) ? (colors.blue / normSum) : 0.0;

        int rawSum = Math.max(0, rawR) + Math.max(0, rawG) + Math.max(0, rawB);
        double rawNormR = (rawSum > 0) ? ((double) rawR / (double) rawSum) : 0.0;
        double rawNormG = (rawSum > 0) ? ((double) rawG / (double) rawSum) : 0.0;
        double rawNormB = (rawSum > 0) ? ((double) rawB / (double) rawSum) : 0.0;

        // Also put something easy-to-find in the rlog.
        Logger.recordOutput("Test/ColorDistance/AcquisitionPath", acquisitionPath);
        Logger.recordOutput("Test/ColorDistance/Gain", (double) gain);
        Logger.recordOutput("Test/ColorDistance/LedEnabled", ledEnabled);
        Logger.recordOutput("Test/ColorDistance/SoftwareLedSupported", switchableLight != null);
        Logger.recordOutput("Test/ColorDistance/Normalized/R", (double) colors.red);
        Logger.recordOutput("Test/ColorDistance/Normalized/G", (double) colors.green);
        Logger.recordOutput("Test/ColorDistance/Normalized/B", (double) colors.blue);
        Logger.recordOutput("Test/ColorDistance/Normalized/A", (double) colors.alpha);
        Logger.recordOutput("Test/ColorDistance/ChromaticityNorm/R", normR);
        Logger.recordOutput("Test/ColorDistance/ChromaticityNorm/G", normG);
        Logger.recordOutput("Test/ColorDistance/ChromaticityNorm/B", normB);
        Logger.recordOutput("Test/ColorDistance/Raw/R", rawR);
        Logger.recordOutput("Test/ColorDistance/Raw/G", rawG);
        Logger.recordOutput("Test/ColorDistance/Raw/B", rawB);
        Logger.recordOutput("Test/ColorDistance/Raw/A", rawA);
        Logger.recordOutput("Test/ColorDistance/ChromaticityRaw/R", rawNormR);
        Logger.recordOutput("Test/ColorDistance/ChromaticityRaw/G", rawNormG);
        Logger.recordOutput("Test/ColorDistance/ChromaticityRaw/B", rawNormB);
        Logger.recordOutput("Test/ColorDistance/DistanceIn", distanceIn);
        Logger.recordOutput("Test/ColorDistance/DistanceM", distanceM);

        telemetry.addData("Acquired via", acquisitionPath);
        telemetry.addData("Gain (dpad)", gain);
        telemetry.addData("LED (A)", ledEnabled);
        telemetry.addData("Software LED supported", switchableLight != null);
        telemetry.addData("Distance (in)", distanceIn);
        telemetry.addData("Distance (m)", distanceM);
        telemetry.addData("Raw R", rawR);
        telemetry.addData("Raw G", rawG);
        telemetry.addData("Raw B", rawB);
        telemetry.addData("Raw A", rawA);
        telemetry.addData("Raw norm R", rawNormR);
        telemetry.addData("Raw norm G", rawNormG);
        telemetry.addData("Raw norm B", rawNormB);
        telemetry.addData("Norm R", colors.red);
        telemetry.addData("Norm G", colors.green);
        telemetry.addData("Norm B", colors.blue);
        telemetry.addData("Norm A", colors.alpha);
        telemetry.addData("Norm chroma R", normR);
        telemetry.addData("Norm chroma G", normG);
        telemetry.addData("Norm chroma B", normB);
        telemetry.update();
    }
}
