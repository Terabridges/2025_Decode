package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Vision;

@Disabled
@TeleOp(name="VisionTester", group="Test")
public class VisionTester extends LinearOpMode {

    public Vision vision;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        vision.toInit();

        waitForStart();
        while (opModeIsActive()){

            vision.update();

            updateTelemetry();
        }
    }

    //methods

    private void updateTelemetry()
    {
        joinedTelemetry.addData("X Offset", vision.getTx());
        joinedTelemetry.addData("Y Offset", vision.getTy());
        joinedTelemetry.addData("X Fid", vision.getFiducialX());
        joinedTelemetry.addData("Y Fid", vision.getFiducialY());
        joinedTelemetry.addData("Z Fid", vision.getFiducialZ());
        joinedTelemetry.addData("Distance (inches)", vision.getDistanceInches());
        joinedTelemetry.addData("Planar Distance (inches)", vision.getPlanarDistanceInches());
        joinedTelemetry.addData("Camera Bearing", vision.getCameraBearingDeg());

//        //using field map
//        joinedTelemetry.addData("Bot pose MT1", vision.getBotPoseMT1());
//
//        //using field map + pinpoint
//        joinedTelemetry.addData("Bot pose MT2", vision.getBotPoseMT2());

        joinedTelemetry.update();
    }
}
