package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@Configurable
@TeleOp(name="LockTester", group="Test")
public class LockTester extends LinearOpMode {

    public Vision vision;
    public Shooter shooter;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public PIDController turretController;
    public static double p = 0.0, i = 0.0, d = 0.0;
    public static double turretTarget;
    double turretPower, currentAngle;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap);
        vision.toInit();
        shooter.toInit();

        turretController = new PIDController(p, i, d);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a)
            {
                shooter.toggleTurretLock();
            }

            //shooter.turretLockUpdate(vision.getTx());
            if (shooter.turretLock){
            shooter.turret.setPower(setTurretPID(turretTarget));
            }

            shooter.update();
            vision.update();

            updateTelem();
        }
    }

    //methods

    private void updateTelem()
    {
        joinedTelemetry.addData("X error", vision.getTx());
        joinedTelemetry.addData("Power", turretPower);
        joinedTelemetry.update();
    }

    public double setTurretPID(double targetAngle) {

        //targetAngle is 0, if we want to minimize error
        currentAngle = vision.getTx();
        turretController.setPID(p, i, d);
        turretPower = turretController.calculate(currentAngle, targetAngle);
        turretPower *= 0.1;
        return turretPower;
        //TODO AXON TURRET IS NOT IN CR MODE

    }
}
