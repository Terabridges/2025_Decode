package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
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
    public static double p = 0.022, i = 0.01, d = 0.0;
    public static double turretTarget = 0.0;
    double posTolerance = 1.2;
    public static double inteTolerance = 6;
    public static double maxPow = 0.135;
    public static double deadband = 0.0;
    double turretPower, error;
    boolean useTurretLock = true;
    double manualPower = 0;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);
        vision.toInit();
        shooter.toInit();

        turretController = new PIDController(p, i, d);
        turretController.setIntegrationBounds(-inteTolerance, inteTolerance);
        turretController.setTolerance(posTolerance);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a)
            {
                useTurretLock = !useTurretLock;
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                turretController.reset();
            }

            if (useTurretLock){
            shooter.turret.setPower(setTurretPID(turretTarget));
            } else {
                shooter.turret.setPower(manualPower);
            }

            if (currentGamepad1.left_trigger > 0.1){
                manualPower = 0.4;
            } else if (currentGamepad1.right_trigger > 0.1){
                manualPower = -0.4;
            } else {
                manualPower = 0;
            }

            //shooter.update();
            vision.update();

            updateTelem();
        }
    }

    //methods

    private void updateTelem()
    {
        joinedTelemetry.addData("X error", vision.getTx());
        joinedTelemetry.addData("Power", turretPower);
        joinedTelemetry.addData("LockOn", useTurretLock);
        joinedTelemetry.update();
    }

    public double setTurretPID(double targetAngle) {

        turretController.setPID(p, i, d);
        error = vision.getTx();
        if (Math.abs(error) < deadband) error = 0.0;
        turretPower = turretController.calculate(error, targetAngle);
        turretPower = clamp(turretPower, -maxPow, maxPow);
        return turretPower;

    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}