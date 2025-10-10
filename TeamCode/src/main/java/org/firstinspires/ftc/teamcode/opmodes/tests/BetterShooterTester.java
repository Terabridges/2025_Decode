package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

@Configurable
@TeleOp(name="BetterShooterTester", group="Test")
public class BetterShooterTester extends LinearOpMode {

    public Shooter shooter;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public DcMotor encoderFly;

    public double TICKS_PER_REV = 28.0; // goBILDA 5000-0002-0001
    public boolean flyRun = true;

    public ElapsedTime rpmTimer  = new ElapsedTime();
    public int lastTicks = 0;

    public PIDController shooterController;
    public static double p = 0.0, i = 0.0, d = 0.0;
    public static double shooterTarget;
    double shooterPower;

    double dtRPM, tps, rps, rpm;
    int curPos, difTicks;

    boolean isTouching = false;

    public static double manualPow = 0.0;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        shooter = new Shooter(hardwareMap);

        rpmTimer.reset();
        lastTicks = shooter.flyLeft.getCurrentPosition();
        shooterController = new PIDController(p, i, d);
        encoderFly = shooter.flyLeft;

        //shooterController.setIntegrationBounds(-0.2, 0.2);
        //shooterController.setTolerance(100);

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

            if (flyRun){
                shooterSetPower(setShooterPID(shooterTarget));
            } else {
                shooterSetPower(0);
            }

            isTouching = shooter.hoodSwitch.isPressed();

            updateTelem();
        }
    }

    public double setShooterPID(double targetRPM) {

        shooterController.setPID(p, i, d);
        shooterPower = shooterController.calculate(rpm, targetRPM);
        return shooterPower;

    }

    public void shooterSetPower(double pow){
        pow = Math.max(pow, 0.9);
        pow = Math.min(pow, -0.9);
        shooter.flyLeft.setPower(pow);
        shooter.flyRight.setPower(pow);
    }

    public void updateShooterRPM(){
        dtRPM = Math.max(1e-3, rpmTimer.seconds());
        rpmTimer.reset();
        curPos = encoderFly.getCurrentPosition();
        difTicks = curPos - lastTicks;
        lastTicks = curPos;
        tps = difTicks / dtRPM;
        rps = tps / TICKS_PER_REV;
        rpm = rps * 60.0;
    }

    public void updateTelem(){
        joinedTelemetry.addData("Target RPM", shooterTarget);
        joinedTelemetry.addData("Current RPM", rpm);
        joinedTelemetry.addData("Error", shooterTarget - rpm);
        joinedTelemetry.addData("Switch Activated?", isTouching);
        joinedTelemetry.addData("Left Power", shooter.flyLeft.getPower());
        joinedTelemetry.addData("Right Power", shooter.flyRight.getPower());
        joinedTelemetry.update();
    }
}