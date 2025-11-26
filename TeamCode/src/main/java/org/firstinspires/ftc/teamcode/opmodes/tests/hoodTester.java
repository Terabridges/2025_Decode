package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@Configurable
@TeleOp(name="hoodTester", group="Test")
public class hoodTester extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Shooter shooter;
    public Vision vision;
    public static double hoodDown = 0;
    public static double hoodUp = 1;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a){
                shooter.hood.setPosition(hoodDown);
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                shooter.hood.setPosition(hoodUp);
        }

    }


}
}
