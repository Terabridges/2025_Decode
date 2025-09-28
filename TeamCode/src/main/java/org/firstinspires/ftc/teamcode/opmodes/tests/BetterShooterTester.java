package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

@TeleOp(name="BetterShooterTester", group="Test")
public class BetterShooterTester extends LinearOpMode {

    public Shooter shooter;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    double velocity = 0.0;
    private int desiredRpm = 0;
    boolean flyRun = false;

    private ElapsedTime rpmTimer  = new ElapsedTime();
    private int lastTicks = 0;

    @Override
    public void runOpMode(){

        shooter = new Shooter(hardwareMap);
        rpmTimer.reset();
        lastTicks = shooter.flyLeft.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            velocity = (desiredRpm * TICKS_PER_REV) / 60.0;

            if (currentGamepad1.a && !previousGamepad1.a){
                flyRun = !flyRun;
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                desiredRpm += 1000;
                if (desiredRpm == 7000){
                    desiredRpm = 0;
                }
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                desiredRpm -= 1000;
                if (desiredRpm == -1000){
                    desiredRpm = 6000;
                }
            }

            if (flyRun){
                shooter.flyLeft.setVelocity(velocity);
                shooter.flyRight.setVelocity(velocity);
            } else {
                shooter.flyLeft.setVelocity(0);
                shooter.flyRight.setVelocity(0);
            }

            double dtRpm = rpmTimer.seconds();
            rpmTimer.reset();
            int cur = shooter.flyLeft.getCurrentPosition();
            int d = cur - lastTicks;
            lastTicks = cur;
            //Encoder is on left
            double tps = d / dtRpm; // ticks/sec
            double rps = tps / TICKS_PER_REV;
            double rpm = rps * 60.0;

            telemetry.addData("Desired RPM", desiredRpm);
            telemetry.addData("Left RPM", rpm);
            telemetry.addData("Left Power", shooter.flyLeft.getPower());
            telemetry.addData("Left Velocity", shooter.flyLeft.getVelocity());
            telemetry.addData("Right Power", shooter.flyRight.getPower());
            telemetry.addData("Right Velocity", shooter.flyRight.getVelocity());
            telemetry.update();
        }
    }
}