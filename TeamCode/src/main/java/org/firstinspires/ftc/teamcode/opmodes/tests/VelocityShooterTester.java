package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

@Configurable
@TeleOp(name="VelocityTester", group="Test")
public class VelocityShooterTester extends LinearOpMode {

    public Shooter shooter;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    private VoltageSensor battery;
    public int selectedRunType = 0;
    //0=none, 1=left, 2=right, 3=both


    private JoinedTelemetry joinedTelemetry;

    public static double targetRPM = 0.0;

    double leftRPM, rightRPM, targetVelocity;

    double ticksPerRev = 28;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        shooter = new Shooter(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            if (currentGamepad1.a && !previousGamepad1.a){
                selectedRunType = 0;
            }

            if (currentGamepad1.x && !previousGamepad1.y){
                selectedRunType = 1;
            }

            if (currentGamepad1.b && !previousGamepad1.b){
                selectedRunType = 2;
            }

            if (currentGamepad1.y && !previousGamepad1.y){
                selectedRunType = 3;
            }

            updateTelem();
            updateVelocities(targetRPM);
        }
    }

    public void updateTelem(){

        leftRPM = velToRPM(shooter.flyLeft.getVelocity());
        rightRPM = velToRPM(shooter.flyRight.getVelocity());

        joinedTelemetry.addData("Left RPM", leftRPM);
        joinedTelemetry.addData("Right RPM", rightRPM);
        joinedTelemetry.addData("Target RPM", targetRPM);
        joinedTelemetry.addData("Selected", selectedRunType);
        joinedTelemetry.update();
    }

    public void updateVelocities(double RPM){
        targetVelocity = RPMToVel(RPM);

        if (selectedRunType == 1){
            shooter.flyLeft.setVelocity(targetVelocity);
        } else if (selectedRunType == 2){
            shooter.flyRight.setVelocity(targetVelocity);
        } else if (selectedRunType == 3){
            shooter.flyLeft.setVelocity(targetVelocity);
            shooter.flyRight.setVelocity(targetVelocity);
        }
    }

    public double velToRPM(double tps){
        double rps = tps / ticksPerRev;
        return rps*60;
    }

    public double RPMToVel(double rpm){
        double tpm = rpm * ticksPerRev;
        return tpm/60;
    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}