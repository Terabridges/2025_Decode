package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Vision;

@Disabled
@Configurable
@TeleOp(name="TurretTester", group="Test")
public class TurretTester extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Shooter shooter;
    public Vision vision;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    double turretManualPow = 0;

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

            if (currentGamepad1.right_trigger > 0.05){
                turretManualPow = currentGamepad1.right_trigger;
            } else if (currentGamepad1.left_trigger > 0.05){
                turretManualPow = -currentGamepad1.left_trigger;
            } else {
                turretManualPow = 0;
            }

            if(currentGamepad1.a && !previousGamepad1.a){
                shooter.light.setPosition(0.470);
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                shooter.light.setPosition(0.280);
            }

            if (!pastPosLimit() && turretManualPow > 0) {
                shooter.setTurretPower(turretManualPow);
            } else if (!pastNegLimit() && turretManualPow < 0) {
                shooter.setTurretPower(turretManualPow);
            } else {
                shooter.setTurretPower(0);
            }

            joinedTelemetry.addData("TurretPos", shooter.turretEnc.getCurrentPosition());
            joinedTelemetry.addData("Turret Manual Pow", turretManualPow);
            joinedTelemetry.addData("Past pos lim", (shooter.turretEnc.getCurrentPosition() <= 100));
            joinedTelemetry.addData("neg pos lim", (shooter.turretEnc.getCurrentPosition() >= 325));
            joinedTelemetry.update();

        }

    }

    boolean pastPosLimit(){
        return (shooter.turretEnc.getCurrentPosition() <= 100);
    }

    boolean pastNegLimit(){
        return (shooter.turretEnc.getCurrentPosition() >= 325);
    }

    //left (negative power) makes enc go up
    //Turret limits: posLim:28 - negLim:350
    //Really: 30 - 340
    //Start limiting 130 - 240

}
