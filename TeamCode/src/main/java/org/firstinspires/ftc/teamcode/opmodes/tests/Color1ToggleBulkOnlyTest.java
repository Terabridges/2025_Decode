package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "Color1 Toggle Read (PsiKit BulkOnly)", group = "Benchmark")
public class Color1ToggleBulkOnlyTest extends OpMode {

    private NormalizedColorSensor color1Sensor;
    private NormalizedColorSensor color2Sensor;
    private NormalizedColorSensor color3Sensor;
    private GoBildaPinpointDriver pinpoint;

    private boolean readColor1Enabled;
    private boolean readColor2Enabled;
    private boolean readColor3Enabled;

    private boolean previousA;
    private boolean previousB;
    private boolean previousX;

    private boolean previousBulkOnly;
    private boolean previousColorBackgroundPolling;

    @Override
    public void init() {
        previousBulkOnly = FtcLogTuning.bulkOnlyLogging;
        previousColorBackgroundPolling = FtcLogTuning.processColorDistanceSensorsInBackground;

        FtcLogTuning.bulkOnlyLogging = true;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;

        try {
            color1Sensor = hardwareMap.get(NormalizedColorSensor.class, "color1");
        } catch (Exception ignored) {
            color1Sensor = null;
        }

        try {
            color2Sensor = hardwareMap.get(NormalizedColorSensor.class, "color2");
        } catch (Exception ignored) {
            color2Sensor = null;
        }

        try {
            color3Sensor = hardwareMap.get(NormalizedColorSensor.class, "color3");
        } catch (Exception ignored) {
            color3Sensor = null;
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception ignored) {
            pinpoint = null;
        }

        readColor1Enabled = false;
        readColor2Enabled = false;
        readColor3Enabled = false;

        previousA = false;
        previousB = false;
        previousX = false;
    }

    @Override
    public void loop() {
        double colorReadTotalUs = 0.0;

        boolean currentA = gamepad1.a;
        if (currentA && !previousA) {
            readColor1Enabled = !readColor1Enabled;
        }
        previousA = currentA;

        boolean currentB = gamepad1.b;
        if (currentB && !previousB) {
            readColor2Enabled = !readColor2Enabled;
        }
        previousB = currentB;

        boolean currentX = gamepad1.x;
        if (currentX && !previousX) {
            readColor3Enabled = !readColor3Enabled;
        }
        previousX = currentX;

        Logger.recordOutput("ColorTest/ReadEnabled/color1", readColor1Enabled);
        Logger.recordOutput("ColorTest/ReadEnabled/color2", readColor2Enabled);
        Logger.recordOutput("ColorTest/ReadEnabled/color3", readColor3Enabled);

        if (pinpoint != null) {
            final long ppStartNs = System.nanoTime();
            pinpoint.update();
            final long ppEndNs = System.nanoTime();

            final double xMm = pinpoint.getPosX(DistanceUnit.MM);
            final double yMm = pinpoint.getPosY(DistanceUnit.MM);
            final double headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            Logger.recordOutput("ColorTest/Pinpoint/Xmm", xMm);
            Logger.recordOutput("ColorTest/Pinpoint/Ymm", yMm);
            Logger.recordOutput("ColorTest/Pinpoint/HeadingDeg", headingDeg);
            Logger.recordOutput("ColorTest/Pinpoint/UpdateUs", (ppEndNs - ppStartNs) / 1_000.0);

            telemetry.addData("pinpoint x mm", xMm);
            telemetry.addData("pinpoint y mm", yMm);
            telemetry.addData("pinpoint heading deg", headingDeg);
        }

        if (readColor1Enabled && color1Sensor != null) {
            final long color1StartNs = System.nanoTime();
            NormalizedRGBA colors = color1Sensor.getNormalizedColors();
            final long color1EndNs = System.nanoTime();
            final double color1ReadUs = (color1EndNs - color1StartNs) / 1_000.0;
            colorReadTotalUs += color1ReadUs;

            Logger.recordOutput("ColorTest/color1/R", colors.red);
            Logger.recordOutput("ColorTest/color1/G", colors.green);
            Logger.recordOutput("ColorTest/color1/B", colors.blue);
            Logger.recordOutput("ColorTest/color1/A", colors.alpha);
            Logger.recordOutput("ColorTest/TimingUs/color1Read", color1ReadUs);

            telemetry.addData("color1 r", colors.red);
            telemetry.addData("color1 g", colors.green);
            telemetry.addData("color1 b", colors.blue);
            telemetry.addData("color1 a", colors.alpha);
        }

        if (readColor2Enabled && color2Sensor != null) {
            final long color2StartNs = System.nanoTime();
            NormalizedRGBA colors = color2Sensor.getNormalizedColors();
            final long color2EndNs = System.nanoTime();
            final double color2ReadUs = (color2EndNs - color2StartNs) / 1_000.0;
            colorReadTotalUs += color2ReadUs;

            Logger.recordOutput("ColorTest/color2/R", colors.red);
            Logger.recordOutput("ColorTest/color2/G", colors.green);
            Logger.recordOutput("ColorTest/color2/B", colors.blue);
            Logger.recordOutput("ColorTest/color2/A", colors.alpha);
            Logger.recordOutput("ColorTest/TimingUs/color2Read", color2ReadUs);

            telemetry.addData("color2 r", colors.red);
            telemetry.addData("color2 g", colors.green);
            telemetry.addData("color2 b", colors.blue);
            telemetry.addData("color2 a", colors.alpha);
        }

        if (readColor3Enabled && color3Sensor != null) {
            final long color3StartNs = System.nanoTime();
            NormalizedRGBA colors = color3Sensor.getNormalizedColors();
            final long color3EndNs = System.nanoTime();
            final double color3ReadUs = (color3EndNs - color3StartNs) / 1_000.0;
            colorReadTotalUs += color3ReadUs;

            Logger.recordOutput("ColorTest/color3/R", colors.red);
            Logger.recordOutput("ColorTest/color3/G", colors.green);
            Logger.recordOutput("ColorTest/color3/B", colors.blue);
            Logger.recordOutput("ColorTest/color3/A", colors.alpha);
            Logger.recordOutput("ColorTest/TimingUs/color3Read", color3ReadUs);

            telemetry.addData("color3 r", colors.red);
            telemetry.addData("color3 g", colors.green);
            telemetry.addData("color3 b", colors.blue);
            telemetry.addData("color3 a", colors.alpha);
        }

        telemetry.addData("BulkOnly", FtcLogTuning.bulkOnlyLogging);
        Logger.recordOutput("ColorTest/TimingUs/ColorReadTotal", colorReadTotalUs);
        telemetry.addData("Read color1 (A)", readColor1Enabled);
        telemetry.addData("Read color2 (B)", readColor2Enabled);
        telemetry.addData("Read color3 (X)", readColor3Enabled);
        telemetry.addData("color1 found", color1Sensor != null);
        telemetry.addData("color2 found", color2Sensor != null);
        telemetry.addData("color3 found", color3Sensor != null);
        telemetry.addData("pinpoint found", pinpoint != null);
        telemetry.update();
    }

    @Override
    public void stop() {
        FtcLogTuning.bulkOnlyLogging = previousBulkOnly;
        FtcLogTuning.processColorDistanceSensorsInBackground = previousColorBackgroundPolling;
    }
}
