package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.arcrobotics.ftclib.controller.PIDController;
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
@TeleOp(name="TurretCoarsePid", group="Test")
public class TurretCoarsePid extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Vision vision;
    public Shooter shooter;

    public PIDController turretController;
    public static double p = 0.006, i = 0.005, d = 0.0;
    public static double posTolerance = 7;
    public static double inteTolerance = 15;
    public static double turretPower = 0.0;
    public static double turretTarget = 0.0;
    public static double max = 1;
    public static double manualPow;
    boolean usePid = true;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        shooter = new Shooter(hardwareMap, vision);

        turretController = new PIDController(p, i, d);
        turretController.setIntegrationBounds(-inteTolerance, inteTolerance);
        turretController.setTolerance(posTolerance);

        waitForStart();
        while (opModeIsActive()){

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a) {
                usePid = !usePid;
            }

            joinedTelemetry.addData("Current Pos", shooter.turretEnc.getCurrentPosition());
            joinedTelemetry.addData("Target Pos", turretTarget);
            joinedTelemetry.addData("power", -turretPower);
            joinedTelemetry.addData("usePid", usePid);
            joinedTelemetry.addData("manual pow", manualPow);
            joinedTelemetry.update();

            if (usePid) {
                shooter.turret.setPower(setTurretPID(turretTarget));
            } else {
                shooter.turret.setPower(manualPow);
            }

            if (gamepad1.right_trigger > 0){
                manualPow = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0){
                manualPow = -gamepad1.left_trigger;
            } else {
                manualPow = 0;
            }
        }

    }


    public double setTurretPID(double targetPos) {
        turretController.setPID(p, i, d);
        double currentPos = shooter.turretEnc.getCurrentPosition();
        turretPower = turretController.calculate(currentPos, targetPos);
        turretPower = clamp(turretPower, -max, max);
        return -turretPower;
    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

}
