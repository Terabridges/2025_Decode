package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@Configurable
@TeleOp(name="LockTester", group="Test")
public class LockTester extends LinearOpMode {

    public Vision vision;
    public Shooter shooter;
    public Drive drive;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public PIDController outerTurretController;
    public static double p2 = 0.005, i2 = 0.012, d2 = 0.0;
    public static double outerTurretTarget = 0.0;
    public static double inteTolerance2 = 8;
    public static double maxPow2 = 0.14;
    public static double minPow2 = 0.07;
    public static double deadband2 = 0.0;
    double outerTurretPower, error;
    boolean useTurretLock = true;
    double manualPower = 0;
    public static double lowThresh = 0.01;

//    public PIDController innerTurretController;
//    public static double p1 = 0.00, i1 = 0.0, d1 = 0.0;
//    public static double innerTurretTarget = 0.0;
//    public static double inteTolerance1 = 8;
//    public static double maxPow1 = 0.14;
//    public static double deadband1 = 0.0;
//    double innerTurretPower;

    //public static double innerThresh = 6;

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

        outerTurretController = new PIDController(p2, i2, d2);
        outerTurretController.setIntegrationBounds(-inteTolerance2, inteTolerance2);

//        innerTurretController = new PIDController(p1, i1, d1);
//        innerTurretController.setIntegrationBounds(-inteTolerance1, inteTolerance1);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a)
            {
                useTurretLock = !useTurretLock;
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                outerTurretController.reset();
            }

            if (useTurretLock){
//                if (vision.getTx() < innerThresh) {
//                    shooter.turret.setPower(setInnerTurretPID(innerTurretTarget));
//                } else {
//                    shooter.turret.setPower(setOuterTurretPID(outerTurretTarget));
//                }
                shooter.turret.setPower(setOuterTurretPID(outerTurretTarget));
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

            //shooter.update();
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
        joinedTelemetry.addData("Power", outerTurretPower);
        joinedTelemetry.addData("LockOn", useTurretLock);
        joinedTelemetry.addData("Manual Power", manualPower);
        joinedTelemetry.update();
    }

    public double setOuterTurretPID(double targetAngle) {

        outerTurretController.setPID(p2, i2, d2);
        error = vision.getTx();
        if (Math.abs(error) < deadband2) error = 0.0;
        outerTurretPower = outerTurretController.calculate(error, targetAngle);
        outerTurretPower = clamp(outerTurretPower, -maxPow2, maxPow2);
        if (outerTurretPower > lowThresh){
            if (outerTurretPower < minPow2){
                outerTurretPower = minPow2;
            }
        } else if (outerTurretPower < -lowThresh){
            if (outerTurretPower > -minPow2){
                outerTurretPower = -minPow2;
            }
        }
        return outerTurretPower;

    }

//    public double setInnerTurretPID(double targetAngle) {
//
//        innerTurretController.setPID(p1, i1, d1);
//        error = vision.getTx();
//        if (Math.abs(error) < deadband1) error = 0.0;
//        innerTurretPower = innerTurretController.calculate(error, targetAngle);
//        innerTurretPower = clamp(innerTurretPower, -maxPow1, maxPow1);
//        return innerTurretPower;
//
//    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}