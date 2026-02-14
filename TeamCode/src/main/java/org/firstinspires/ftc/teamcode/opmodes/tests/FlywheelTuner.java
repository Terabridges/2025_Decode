package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="FlywheelTuner", group="Test")
public class FlywheelTuner extends OpMode {

    Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

    }

    @Override
    public void start(){
        //robot.toInit();
        robot.outtake.shooter.toInit();
    }

    @Override
    public void loop() {
        //robot.update();
        robot.outtake.shooter.update();
//        robot.outtake.shooter.setLeftFlywheelPow(gamepad1.left_trigger);
//        robot.outtake.shooter.setRightFlywheelPow(gamepad1.right_trigger);

        joinedTelemetry.addData("Current RPM", robot.outtake.shooter.getCurrentRPM());
        joinedTelemetry.addData("Target RPM", robot.outtake.shooter.getTargetRPM());
        joinedTelemetry.addData("Current Power", robot.outtake.shooter.getCurrentPower());
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}
