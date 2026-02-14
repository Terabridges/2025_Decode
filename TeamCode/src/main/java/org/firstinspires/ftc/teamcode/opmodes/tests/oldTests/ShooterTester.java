package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Vision;

@Disabled
//@Configurable
@TeleOp(name="ShooterTester", group="Test")
public class ShooterTester extends LinearOpMode {

    public Shooter shooter;
    public Vision vision;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public DcMotor encoderFly;

    public double TICKS_PER_REV = 28.0; // goBILDA 5000-0002-0001
    public boolean flyRun = true;

    public ElapsedTime rpmTimer  = new ElapsedTime();
    public int lastTicks = 0;

    public PIDController shooterController;
    public static double p = 0.0, i = 0.0, d = 0.0;
    public static int shooterTarget;
    double pid, shooterPower;

    double dtRPM, tps, rps, rpm;
    int curPos, difTicks;

    @Override
    public void runOpMode(){

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);
        rpmTimer.reset();
        lastTicks = shooter.flyLeft.getCurrentPosition();
        shooterController = new PIDController(p, i, d);
        encoderFly = shooter.flyLeft;

        shooterController.setPID(p, i, d);
        //shooterController.setIntegrationBounds(-0.2, 0.2);
        //shooterController.setTolerance(100);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            updateShooterRPM();

            if (flyRun){
                shooterSetPower(setShooterPID(shooterTarget));
            } else {
                shooterSetPower(0);
            }

            updateTelem();
        }
    }

    public double setShooterPID(double targetRPM) {


        pid = shooterController.calculate(rpm, targetRPM);

        shooterPower = pid;
        shooterPower = Math.max(-1, Math.min(1, shooterPower));

        return shooterPower;
    }

    public void shooterSetPower(double pow){
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
        
    }
}