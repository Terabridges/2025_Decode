package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="DistanceSensor", group="Test")
public class DistanceSensorTest extends LinearOpMode {

    private AnalogInput distanceSensor;
    private double maxVolts = 3.3;
    //private double maxDistanceMM = 1000;

    @Override
    public void runOpMode(){

        distanceSensor = hardwareMap.get(AnalogInput.class, "distance_sensor");


        waitForStart();
        while (opModeIsActive()){

            double volts = distanceSensor.getVoltage();
            //double distance = (volts / maxVolts) * maxDistanceMM;

            telemetry.addData("Volts", volts);
            //telemetry.addData("Distance", distance);

        }

    }
}
