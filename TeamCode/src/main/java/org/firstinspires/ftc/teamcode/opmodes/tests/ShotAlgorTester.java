package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utility.ShotAlgorithm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
@TeleOp(name="ShotAlgorithmTester", group="Test")
public class ShotAlgorTester extends LinearOpMode{

    public Shooter shooter;
    public Vision vision;
    public ShotAlgorithm shotAlgor;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    private VoltageSensor battery;
    public int selectedRunType = 0;
    //0=none, 1=left, 2=right, 3=both


    private JoinedTelemetry joinedTelemetry;

    public static double targetRPM = 0.0;

    double leftRPM, rightRPM, targetVelocity;

    double ticksPerRev = 28;



    double x = 0;   // m
    double y = 1.143 - 0.3048; // m
    double ux = 0; // m/s (positive toward goal)
    double uy = 0;
    double thetaMin = Math.toRadians(0);
    double thetaMax = Math.toRadians(195);
    double vMax = 25.0; // m/s (example)


    public PIDController turretController;
    public static double p = 0.02, i = 0.00015, d = 0.0009;
    public static double turretTarget = 0.0;
    double posTolerance = 1.2;
    double velTolerance = 5.0;
    public static double inteTolerance = 6.0;
    public static double maxPow = 0.6;
    public static double deadband = 0.18;
    double turretPower, error;
    boolean useTurretLock = true;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);
        shotAlgor = new ShotAlgorithm(thetaMin, thetaMax, vMax);
        battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            x = vision.getDistanceInches();

            if (useTurretLock){
                shooter.turret.setPower(setTurretPID(turretTarget));
            }

            if (currentGamepad1.a && !previousGamepad1.y){
                selectedRunType = 1;
            }

            if (currentGamepad1.b && !previousGamepad1.b){
                selectedRunType = 2;
            }

            if (currentGamepad1.y && !previousGamepad1.y){
                useTurretLock = !useTurretLock;
            }

            updateTelem();
            updateVelocities();
        }
    }

    public double setTurretPID(double targetAngle) {

        turretController.setPID(p, i, d);
        error = vision.getTx();
        if (Math.abs(error) < deadband) error = 0.0;
        turretPower = turretController.calculate(error, targetAngle);
        turretPower = clamp(turretPower, -maxPow, maxPow);
        return turretPower;

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

    public void updateVelocities(){

        if (selectedRunType == 1){

            ShotAlgorithm.ShotSolution bestShot = shotAlgor.findBestShot(x,y,ux,uy,thetaMin,thetaMax,vMax, 200);

            if (bestShot != null) {
                targetRPM = shotAlgor.velocityToRPM(bestShot.v);
                shooter.setHoodPos(bestShot.theta); //set hood angle
                shooter.setShooterRPM(targetRPM); //set RPM
            } else {
                // no feasible shot in constraints
            }

        } else if (selectedRunType == 2){
            targetRPM = 0;
            shooter.setShooterVel(targetRPM);
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
