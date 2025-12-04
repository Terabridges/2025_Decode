package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.graphics.Color;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;

@Disabled
@Configurable
@TeleOp(name="ColorSensorTester", group="Test")
public class colorSensorTester extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Transfer transfer;
    NormalizedRGBA colors;
    float red;
    float green;
    float blue;
    double distance;
    boolean ballDetected = false;
    boolean seesRed = false;
    String ballColor = "none";

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        transfer = new Transfer(hardwareMap);
        colors = new NormalizedRGBA();

        waitForStart();
        transfer.toInit();
        while (opModeIsActive()){

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            colors = transfer.colorSensor.getNormalizedColors();
            red = colors.red;
            green = colors.green;
            blue = colors.blue;
            distance = transfer.colorSensor.getDistance(DistanceUnit.INCH);

//            if (distance < 1.92){
//                ballDetected = true;
//                if (green > red && green > blue){
//                    ballColor = "green";
//                } else {
//                    ballColor = "purple";
//                }
//            } else {
//                ballDetected = false;
//                ballColor = "none";
//            }

            if(red > green && red > blue){
                seesRed = true;
            } else {
                seesRed = false;
            }

            joinedTelemetry.addData("Red", red);
            joinedTelemetry.addData("Blue", blue);
            joinedTelemetry.addData("Green", green);
            joinedTelemetry.addData("Distance", distance);
//            joinedTelemetry.addData("Sees Ball?", ballDetected);
//            joinedTelemetry.addData("Detected Ball Color?", ballColor);
            joinedTelemetry.addData("Sees Red", seesRed);
            joinedTelemetry.addData("SpindexPos", transfer.spindex.getCurrentPosition());

            joinedTelemetry.update();

        }

    }

}
