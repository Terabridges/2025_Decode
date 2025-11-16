package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;

@Configurable
@TeleOp(name="spindexTester", group="Test")
public class spindexTester extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Transfer transfer;

    public PIDController spindexController;
    public static double p = 0.0017, i = 0.0000125, d = 0.0;
    public static double posTolerance = 5;
    public static double inteTolerance = 5;
    public static double spindexPower = 0.0;
    public static double spindexTarget = 0.0;
    public static double max = 0.225;
    public static int ball = 180;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        transfer = new Transfer(hardwareMap);

        spindexController = new PIDController(p, i, d);
        spindexController.setIntegrationBounds(-inteTolerance, inteTolerance);
        spindexController.setTolerance(posTolerance);

        waitForStart();
        transfer.toInit();
        while (opModeIsActive()){

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.x && !previousGamepad1.x){
                spindexTarget -= ball;
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                spindexTarget += ball;
            }

            joinedTelemetry.addData("Current Pos", transfer.spindex.getCurrentPosition());
            joinedTelemetry.addData("Target Pos", spindexTarget);
            joinedTelemetry.addData("power", spindexPower);
            joinedTelemetry.update();

            transfer.spindex.setPower(setSpindexPID(spindexTarget));
        }

    }

    public double setSpindexPID(double targetPos) {
        spindexController.setPID(p, i, d);
        double currentPos = transfer.spindex.getCurrentPosition();
        spindexPower = spindexController.calculate(currentPos, targetPos);
        spindexPower = clamp(spindexPower, -max, max);
        return spindexPower;
    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

}
