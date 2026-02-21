package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "ColorSensorDecodeTester", group = "Test")
public class ColorSensorDecodeTester extends OpMode {

    public static double detectDistanceInches = 1.55;
    public static double minSaturation = 0.20;
    public static double minValue = 0.012;
    public static double greenRatioThreshold = 1.15;
    public static double purpleRatioThreshold = 1.10;
    public static int minSamplesForDecision = 3;

    private RevColorSensorV3 colorSensor;

    public double red = 0.0;
    public double green = 0.0;
    public double blue = 0.0;
    public double alpha = 0.0;
    public double distanceInches = 0.0;

    public double hueDeg = 0.0;
    public double saturation = 0.0;
    public double value = 0.0;
    public double greenRatio = 0.0;
    public double purpleRatio = 0.0;

    public int logicArtifactPresent = 0;
    public int logicArtifactEnterPulse = 0;
    public int logicArtifactExitPulse = 0;
    public int logicCurrentColorClass = 0;
    public int logicLastPassColorClass = 0;

    public int passCountTotal = 0;
    public int passCountGreen = 0;
    public int passCountPurple = 0;

    private boolean wasArtifactPresent = false;
    private int presentGreenVotes = 0;
    private int presentPurpleVotes = 0;
    private int presentSamples = 0;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        Logger.recordMetadata("OpMode", "ColorSensorDecodeTester");
    }

    @Override
    public void loop() {
        readSensor();
        updateStateMachine();
        logOutputs();
        publishTelemetry();
    }

    private void readSensor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        red = colors.red;
        green = colors.green;
        blue = colors.blue;
        alpha = colors.alpha;
        distanceInches = colorSensor.getDistance(DistanceUnit.INCH);

        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        hueDeg = hsv[0];
        saturation = hsv[1];
        value = hsv[2];

        double epsilon = 1e-6;
        greenRatio = (green + epsilon) / ((red + blue) * 0.5 + epsilon);
        purpleRatio = ((red + blue) * 0.5 + epsilon) / (green + epsilon);
    }

    private void updateStateMachine() {
        boolean artifactPresentNow = distanceInches < detectDistanceInches;

        logicArtifactPresent = artifactPresentNow ? 1 : 0;
        logicArtifactEnterPulse = 0;
        logicArtifactExitPulse = 0;

        logicCurrentColorClass = classifyCurrentSample();

        if (artifactPresentNow && !wasArtifactPresent) {
            logicArtifactEnterPulse = 1;
            presentGreenVotes = 0;
            presentPurpleVotes = 0;
            presentSamples = 0;
        }

        if (artifactPresentNow) {
            if (logicCurrentColorClass == 1) {
                presentGreenVotes++;
            } else if (logicCurrentColorClass == 2) {
                presentPurpleVotes++;
            }
            presentSamples++;
        }

        if (!artifactPresentNow && wasArtifactPresent) {
            logicArtifactExitPulse = 1;
            logicLastPassColorClass = decidePassColorClass();
            passCountTotal++;
            if (logicLastPassColorClass == 1) {
                passCountGreen++;
            } else if (logicLastPassColorClass == 2) {
                passCountPurple++;
            }
        }

        wasArtifactPresent = artifactPresentNow;
    }

    private int classifyCurrentSample() {
        if (saturation < minSaturation || value < minValue) {
            return 0;
        }

        int greenVotes = 0;
        int purpleVotes = 0;

        if (hueDeg >= 80.0 && hueDeg <= 170.0) {
            greenVotes++;
        }
        if (hueDeg >= 250.0 && hueDeg <= 340.0) {
            purpleVotes++;
        }

        if (greenRatio > greenRatioThreshold) {
            greenVotes++;
        }
        if (purpleRatio > purpleRatioThreshold) {
            purpleVotes++;
        }

        if (greenVotes > purpleVotes) {
            return 1;
        }
        if (purpleVotes > greenVotes) {
            return 2;
        }
        return 0;
    }

    private int decidePassColorClass() {
        if (presentSamples < minSamplesForDecision) {
            return 0;
        }

        if (presentGreenVotes > presentPurpleVotes) {
            return 1;
        }
        if (presentPurpleVotes > presentGreenVotes) {
            return 2;
        }
        return 0;
    }

    private void logOutputs() {
        Logger.recordOutput("ColorTest/Raw/Red", red);
        Logger.recordOutput("ColorTest/Raw/Green", green);
        Logger.recordOutput("ColorTest/Raw/Blue", blue);
        Logger.recordOutput("ColorTest/Raw/Alpha", alpha);
        Logger.recordOutput("ColorTest/Raw/DistanceInches", distanceInches);

        Logger.recordOutput("ColorTest/Derived/HueDeg", hueDeg);
        Logger.recordOutput("ColorTest/Derived/Saturation", saturation);
        Logger.recordOutput("ColorTest/Derived/Value", value);
        Logger.recordOutput("ColorTest/Derived/GreenRatio", greenRatio);
        Logger.recordOutput("ColorTest/Derived/PurpleRatio", purpleRatio);

        Logger.recordOutput("ColorTest/Logic/ArtifactPresent", logicArtifactPresent);
        Logger.recordOutput("ColorTest/Logic/ArtifactEnterPulse", logicArtifactEnterPulse);
        Logger.recordOutput("ColorTest/Logic/ArtifactExitPulse", logicArtifactExitPulse);
        Logger.recordOutput("ColorTest/Logic/CurrentColorClass", logicCurrentColorClass);
        Logger.recordOutput("ColorTest/Logic/LastPassColorClass", logicLastPassColorClass);

        Logger.recordOutput("ColorTest/Counts/Total", passCountTotal);
        Logger.recordOutput("ColorTest/Counts/Green", passCountGreen);
        Logger.recordOutput("ColorTest/Counts/Purple", passCountPurple);
        Logger.recordOutput("ColorTest/Debug/PresentSamples", presentSamples);
        Logger.recordOutput("ColorTest/Debug/PresentGreenVotes", presentGreenVotes);
        Logger.recordOutput("ColorTest/Debug/PresentPurpleVotes", presentPurpleVotes);
    }

    private void publishTelemetry() {
        telemetry.addData("distanceIn", distanceInches);
        telemetry.addData("rgb", "%.4f %.4f %.4f", red, green, blue);
        telemetry.addData("alpha", alpha);
        telemetry.addData("hsv", "%.1f %.3f %.3f", hueDeg, saturation, value);
        telemetry.addData("ratios", "g=%.3f p=%.3f", greenRatio, purpleRatio);
        telemetry.addData("artifact", logicArtifactPresent);
        telemetry.addData("currentClass", className(logicCurrentColorClass));
        telemetry.addData("lastPassClass", className(logicLastPassColorClass));
        telemetry.addData("counts", "total=%d green=%d purple=%d", passCountTotal, passCountGreen, passCountPurple);
        telemetry.update();
    }

    private String className(int colorClass) {
        if (colorClass == 1) {
            return "green";
        }
        if (colorClass == 2) {
            return "purple";
        }
        return "unknown";
    }
}
