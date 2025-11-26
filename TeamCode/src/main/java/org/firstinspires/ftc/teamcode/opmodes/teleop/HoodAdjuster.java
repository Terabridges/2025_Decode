package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
@TeleOp(name="HoodAdjuster", group="TeleOp")
public class HoodAdjuster extends LinearOpMode {

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Shooter shooter;
    public Vision vision;

    @Override
    public void runOpMode(){
        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a){
                shooter.setHoodPos(0);
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                shooter.setHoodPos(0.9);
            }
        }

    }
}
