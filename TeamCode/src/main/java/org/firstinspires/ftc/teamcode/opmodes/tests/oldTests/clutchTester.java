package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Transfer;

@Disabled
//@Configurable
@TeleOp(name="clutchTester", group="Test")
public class clutchTester extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Transfer transfer;
    public static double transferUp = 0.05;
    public static double transferDown = 0.85;

//    double clutchUp = 0.17;
//    double clutchBarelyDown = 0.29;
//    double clutchDown = 0.5;
//    double clutchDownFar = 0.85;
    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        transfer = new Transfer(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a){
                transfer.clutch.setPosition(transferDown);
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                transfer.clutch.setPosition(transferUp);
            }
        }

    }


    //Up: 0.05
    //Down: 0.45
    //Really Down: 0.9
    //barely down: 0.35

}
