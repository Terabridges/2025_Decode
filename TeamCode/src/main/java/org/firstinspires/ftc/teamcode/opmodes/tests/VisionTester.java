package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@TeleOp(name="VisionTester", group="Test")
public class VisionTester extends LinearOpMode {

    public Vision vision;

    @Override
    public void runOpMode(){

        vision = new Vision(hardwareMap);
        vision.toInit();

        waitForStart();
        while (opModeIsActive()){

            vision.update();

            telemetry.addData("X Offset", vision.getTx());
            telemetry.addData("Y Offset", vision.getTy());
            telemetry.addData("Distance (inches)", vision.getDistanceInches());
            telemetry.addData("Planar Distance (inches)", vision.getPlanarDistanceInches());
            telemetry.addData("Distance using tan (inches)", vision.getDistanceUsingTan());
//            telemetry.addData("Bot pose MT1", vision.getBotPoseMT1());
//            telemetry.addData("Bot pose MT2", vision.getBotPoseMT2());
            telemetry.update();
        }
    }

    //methods

}
