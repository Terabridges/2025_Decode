package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

@Configurable
@TeleOp(name="IntakeTuner", group="Test")
public class IntakePIDTuner extends LinearOpMode {

    public Intake intake;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    double raiserPower = 0.0;
    public static double raiserTarget = 0.0;
    double currentPos;

    public PIDController raiserController;
    public static double p = 0.007, i = 0.001, d = 0.00005;
    double posTolerance = 5.0;
    double inteTolerance = 8.0;
    public static double leftPow = 0.1;
    public static double rightPow = 0.1;

    boolean useRaiser = false;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        intake = new Intake(hardwareMap);
        raiserController = new PIDController(p, i, d);
        raiserController.setIntegrationBounds(-inteTolerance, inteTolerance);
        raiserController.setTolerance(posTolerance);


        waitForStart();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a){
                useRaiser = !useRaiser;
            }

            if(currentGamepad1.x && !previousGamepad1.x){
                //intake.raiserLeft.setPower(rightPow);
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                intake.raiserRight.setPower(leftPow);
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                //intake.raiserLeft.setPower(0);
                intake.raiserRight.setPower(0);
            }

            if(useRaiser){
                setRaiser(raiserTarget);
            } else {
//                intake.raiserLeft.setPower(0);
//                intake.raiserRight.setPower(0);
            }

            joinedTelemetry.addData("Raiser Pos", intake.raiserEnc.getCurrentPosition());
            joinedTelemetry.addData("Target Pos", raiserTarget);
            joinedTelemetry.addData("Error", raiserTarget-intake.raiserEnc.getCurrentPosition());
            joinedTelemetry.update();
        }
    }

    public double setRaiserPID(double targetPos) {
        raiserController.setPID(p, i, d);
        currentPos = intake.raiserEnc.getCurrentPosition();
        raiserPower = raiserController.calculate(currentPos, targetPos);
        return raiserPower;
    }

    public void setRaiser(double target){
        //intake.raiserLeft.setPower(setRaiserPID(target)); //Was mult by -1, just set servo to be reversed
        intake.raiserRight.setPower(setRaiserPID(target));
    }

    //Intake down: 228 goto 222
    //intake up: 194 goto 190

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}