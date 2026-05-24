package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;
import org.psilynx.psikit.ftc.FtcLogTuning;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="DistanceSensorTest", group="Test")
public class DistanceSensorTest extends OpMode {

    //Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    private AnalogInput frontOuterDistanceSensor;
    private AnalogInput frontInnerDistanceSensor;
    private AnalogInput backOuterDistanceSensor;
    private AnalogInput backInnerDistanceSensor;
    private RevColorSensorV3 frontColor;
    private RevColorSensorV3 middleColor;
    private RevColorSensorV3 backColor;
    private Util util;

    private double frontOuterDistance = 0;
    private double frontInnerDistance = 0;
    private double backOuterDistance = 0;
    private double backInnerDistance = 0;

    public static double frontOuterDistanceLowThresh = 0.48;
    public static double frontInnerDistanceLowThresh = 0.38;
    public static double backOuterDistanceLowThresh = 0.48;
    public static double backInnerDistanceLowThresh = 0.38;

    public boolean frontOuterTripped = false;
    public boolean frontInnerTripped = false;
    public boolean backOuterTripped = false;
    public boolean backInnerTripped = false;

    public static double frontGreenThresh = 0.001; //If green is highest, ball is green
    public static double frontBlueThresh = 0.001; //If blue is highest, ball is purple
    public static double backGreenThresh = 0.001;
    public static double backBlueThresh = 0.001;
    NormalizedRGBA frontColors;
    public float frontRed = 0;
    public float frontGreen = 0;
    public float frontBlue = 0;
    NormalizedRGBA middleColors;
    public float middleRed = 0;
    public float middleGreen = 0;
    public float middleBlue = 0;
    NormalizedRGBA backColors;
    public float backRed = 0;
    public float backGreen = 0;
    public float backBlue = 0;

    private double frontColorDistance = 0;
    private double backColorDistance = 0;

    @Override
    public void init() {
        FtcLogTuning.processColorDistanceSensorsInBackground = true;
        FtcLogTuning.bulkOnlyLogging = false;
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        util = new Util();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        frontOuterDistanceSensor = hardwareMap.get(AnalogInput.class, "distance0");
        frontInnerDistanceSensor = hardwareMap.get(AnalogInput.class, "distance1");
        backOuterDistanceSensor = hardwareMap.get(AnalogInput.class, "distance3");
        backInnerDistanceSensor = hardwareMap.get(AnalogInput.class, "distance2");
        frontColor = hardwareMap.get(RevColorSensorV3.class, "color1");
        middleColor = hardwareMap.get(RevColorSensorV3.class, "color2");
        backColor = hardwareMap.get(RevColorSensorV3.class, "color3");
        frontColors = new NormalizedRGBA();
        middleColors = new NormalizedRGBA();
        backColors = new NormalizedRGBA();

    }

    @Override
    public void start(){
        frontOuterDistance = frontOuterDistanceSensor.getVoltage();
        frontInnerDistance = frontInnerDistanceSensor.getVoltage();
        backOuterDistance = backOuterDistanceSensor.getVoltage();
        backInnerDistance = backInnerDistanceSensor.getVoltage();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        updateDistances();
        updateColorDistances();

        if(currentGamepad1.a && !previousGamepad1.a){
            unTrip();
        }

        joinedTelemetry.addData("FrontOuterTripped", frontOuterTripped);
        joinedTelemetry.addData("FrontInnerTripped", frontInnerTripped);
        joinedTelemetry.addData("BackInnerTripped", backInnerTripped);
        joinedTelemetry.addData("BackOuterTripped", backOuterTripped);
        joinedTelemetry.addData("FrontColorDistance", frontColorDistance);
        joinedTelemetry.addData("BackColorDistance", backColorDistance);
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void updateDistances(){
        frontOuterDistance = frontOuterDistanceSensor.getVoltage();
        frontInnerDistance = frontInnerDistanceSensor.getVoltage();
        backOuterDistance = backOuterDistanceSensor.getVoltage();
        backInnerDistance = backInnerDistanceSensor.getVoltage();

        if (frontOuterDistance < frontOuterDistanceLowThresh){
            frontOuterTripped = true;
        }
        if (frontInnerDistance < frontInnerDistanceLowThresh){
            frontInnerTripped = true;
        }
        if (backOuterDistance < backOuterDistanceLowThresh){
            backOuterTripped = true;
        }
        if (backInnerDistance < backInnerDistanceLowThresh){
            backInnerTripped = true;
        }
    }

    public void unTrip(){
        frontOuterTripped = false;
        frontInnerTripped = false;
        backOuterTripped = false;
        backInnerTripped = false;
    }

    public void updateColorDistances(){
        frontColorDistance = frontColor.getDistance(DistanceUnit.INCH);
        backColorDistance = backColor.getDistance(DistanceUnit.INCH);
    }

    public boolean isFrontColorDistanceTripped(){
        return frontColorDistance > 1.5 && frontColorDistance < 3.2;
    }

    public boolean isBackColorDistanceTripped(){
        return backColorDistance > 1.5 && backColorDistance < 3.2;
    }

}