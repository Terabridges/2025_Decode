package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;

@TeleOp(name="LockTester", group="Test")
public class LockTester extends LinearOpMode {

    public Vision vision;
    public Shooter shooter;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode(){

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap);
        vision.toInit();
        shooter.toInit();

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a)
            {
                shooter.toggleTurretLock();
            }

            shooter.turretLockUpdate(vision.getTx());

            shooter.update();
            vision.update();

            updateTelem();
        }
    }

    //methods

    private void updateTelem()
    {
        telemetry.addData("X Offset", vision.getTx());

        telemetry.update();
    }
}
