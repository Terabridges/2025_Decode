package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.control.Intake.ClutchControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpindexControl;
import org.firstinspires.ftc.teamcode.config.control.Intake.SpinnerControl;
import org.firstinspires.ftc.teamcode.config.control.Other.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.Other.LiftControl;
import org.firstinspires.ftc.teamcode.config.control.Other.LightsControl;
import org.firstinspires.ftc.teamcode.config.control.Other.OtherControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.OuttakeControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.TurretControl;
import org.firstinspires.ftc.teamcode.config.control.Outtake.VisionControl;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.Arrays;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="TransferTest", group="Test")
public class TransferTest extends OpMode {

    Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

    }

    @Override
    public void start(){
        robot.toInit();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        robot.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.a && !previousGamepad1.a){
            robot.intake.clutch.setClutchDown();
        }

        if (currentGamepad1.y && !previousGamepad1.y){
            robot.intake.clutch.setClutchUp();
        }

        robot.intake.spindex.moveSpindexPow(gamepad1.right_trigger*0.2);
        robot.intake.spinner.moveMegaSpinPow(gamepad1.left_stick_x);
    }
}
