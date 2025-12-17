package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@Disabled
@Configurable
@TeleOp(name="BetterShooterTester", group="Test")
public class BetterShooterTester extends LinearOpMode {

    public Shooter shooter;
    public Vision vision;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public DcMotorEx encoderFly;
    private VoltageSensor battery;

    public double TICKS_PER_REV = 28.0; // goBILDA 5000-0002-0001
    public boolean flyRun = false;

    public int lastTicks = 0;

    public PIDController shooterController;
    public static double p = 0.0005, i = 0.0, d = 0.0002;
    public static double shooterTarget;
    double shooterPower;

    public static double kS = 0.05;      // static to overcome friction
    public static double kV = 0.00085;  // ~power per RPM (assume ~5200 RPM ~ 100% power for a 6000RPM motor; adjust)
    public static double kA = 0.0;       // optional accel term if you want extra snap

    public static double manualVel = 1000;
    double rawPower, powerFF;
    double tps, rps, rpm;


    boolean isTouching = false;

    public static double manualPow = 0.6;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);

        shooterController = new PIDController(p, i, d);
        encoderFly = shooter.flyRight;

        lastTicks = encoderFly.getCurrentPosition();
        shooterController.setTolerance(50.0, 500.0);
        shooterController.setIntegrationBounds(-150.0, 150.0);

        java.util.Iterator<VoltageSensor> vsIt = hardwareMap.voltageSensor.iterator();
        if (vsIt.hasNext()) {
            battery = vsIt.next();
        } else {
            battery = null;
            telemetry.addData("Warning","No VoltageSensor found; voltage reporting disabled");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            updateShooterRPM();

            if (currentGamepad1.a && !previousGamepad1.a){
                flyRun = !flyRun;
            }

            if (currentGamepad1.b && !previousGamepad1.b){
                shooterSetPower(manualPow);
            }

            if (currentGamepad1.y && !previousGamepad1.y){
                shooterSetPower(0);
            }

            if (currentGamepad1.x && !previousGamepad1.x){
                encoderFly.setVelocity((manualVel*TICKS_PER_REV)/60);
            }

            if (flyRun){
                shooterSetPower(setShooterPID(shooterTarget));
            }

            //isTouching = shooter.hoodSwitch.isPressed();

            updateTelem();
        }
    }

    public double setShooterPID(double targetRPM) {

        double v = (battery != null) ? battery.getVoltage() : 12.0;
        shooterController.setPID(p, i, d);
        powerFF = kS + (kV * targetRPM);
        powerFF *= (12.0 / Math.max(8.0, v));
        shooterPower = shooterController.calculate(rpm, targetRPM);
        rawPower = powerFF + shooterPower;

        //return rawPower;
        return shooterPower;
    }

    public void shooterSetPower(double pow){
        pow = clamp(pow, -0.975, 0.975);
        shooter.flyLeft.setPower(pow);
        shooter.flyRight.setPower(pow);
    }

    public void updateShooterRPM(){
        tps = encoderFly.getVelocity();
        rps = tps / TICKS_PER_REV;
        rpm = rps * 60.0;
    }

    public void updateTelem(){
        joinedTelemetry.addData("Target RPM", shooterTarget);
        joinedTelemetry.addData("Current RPM", rpm);
        joinedTelemetry.addData("Error", shooterTarget - rpm);
        //joinedTelemetry.addData("Switch Activated?", isTouching);
        joinedTelemetry.addData("Left Power", shooter.flyLeft.getPower());
        joinedTelemetry.addData("Right Power", shooter.flyRight.getPower());
        joinedTelemetry.update();
    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}