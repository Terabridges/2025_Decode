package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Vision;

//@Disabled
//@Configurable
@TeleOp(name="LockTester", group="Test")
public class LockTester extends LinearOpMode {

    public Vision vision;
    public Shooter shooter;
    public Drive drive;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public PIDController turretController;
    public static double p = 0.014, i = 0.012, d = 0.00005;
    public static double turretTarget = 0.0;
    public static double inteTolerance = 6;
    public static double maxPow = 0.18;
    public static double minPow = 0.07;
    public static double deadband = 0.0;
    double turretPower, error;
    boolean useTurretLock = true;
    double manualPower = 0;
    public static double lowThresh = 0.01;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);
        drive = new Drive(hardwareMap);
        vision.toInit();
        shooter.toInit();
        drive.toInit();

        turretController = new PIDController(p, i, d);
        turretController.setIntegrationBounds(-inteTolerance, inteTolerance);

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

            if (currentGamepad1.left_trigger > 0){
                manualPower = currentGamepad1.left_trigger;
            } else if (currentGamepad1.right_trigger > 0){
                manualPower = -currentGamepad1.right_trigger;
            } else {
                manualPower = 0;
            }

            vision.update();

            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -currentGamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = currentGamepad1.left_stick_x;
            double yaw = currentGamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            drive.setDrivePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            drive.update();
            updateTelem();
        }
    }

    //methods

    private void updateTelem()
    {
        joinedTelemetry.addData("X error", vision.getTx());
        joinedTelemetry.addData("Power", turretPower);
        joinedTelemetry.addData("LockOn", useTurretLock);
        joinedTelemetry.addData("Manual Power", manualPower);
        joinedTelemetry.update();
    }

    public double setTurretPID(double targetAngle) {

        turretController.setPID(p, i, d);
        error = vision.getTx();
        if (Math.abs(error) < deadband) error = 0.0;
        turretPower = turretController.calculate(error, targetAngle);
        turretPower = clamp(turretPower, -maxPow, maxPow);
        if (turretPower > lowThresh){
            if (turretPower < minPow){
                turretPower = minPow;
            }
        } else if (turretPower < -lowThresh){
            if (turretPower > -minPow){
                turretPower = -minPow;
            }
        }
        return turretPower;

    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}