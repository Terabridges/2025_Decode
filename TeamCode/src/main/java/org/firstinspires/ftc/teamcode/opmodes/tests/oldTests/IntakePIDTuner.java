package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Intake;

@Disabled
//@Configurable
@TeleOp(name="IntakeTuner", group="Test")
public class IntakePIDTuner extends LinearOpMode {

    public Intake intake;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public AnalogInput port0;
    public AnalogInput port1;
    public AnalogInput port2;
    public AnalogInput port3;


    double raiserPower = 0.0;
    public static double raiserTarget = 0.0;
    double currentPos;

    public PIDController raiserController;
    public static double p = 0.0028, i = 0.0, d = 0.0;
    double posTolerance = 3.0;
    double inteTolerance = 8.0;
    public static double leftPow = 0.1;
    public static double maxPow = 0.15;
    double spinTarget = 0;

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
//        port1 = hardwareMap.get(AnalogInput.class, "analog1");
//        port2 = hardwareMap.get(AnalogInput.class, "intake_analog");
//        port3 = hardwareMap.get(AnalogInput.class, "hood_analog");
//        port0 = hardwareMap.get(AnalogInput.class, "turret_analog");


        waitForStart();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a){
                useRaiser = !useRaiser;
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                intake.raiserRight.setPower(leftPow);
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                intake.raiserRight.setPower(0);
            }

            if (currentGamepad1.left_trigger > 0.1){
                spinTarget = 0.95;
            } else if (currentGamepad1.right_trigger > 0.1){
                spinTarget = -0.95;
            } else {
                spinTarget = 0;
            }

            if(useRaiser){
                setRaiser(raiserTarget);
            }
            intake.spinner.setPower(spinTarget);

            joinedTelemetry.addData("Raiser Pos", intake.raiserEnc.getCurrentPosition());
            joinedTelemetry.addData("Target Pos", raiserTarget);
            joinedTelemetry.addData("Error", raiserTarget-intake.raiserEnc.getCurrentPosition());
            joinedTelemetry.addData("Power", raiserPower);
//            joinedTelemetry.addData("Port0", port0.getVoltage());
//            joinedTelemetry.addData("Port1", port1.getVoltage());
//            joinedTelemetry.addData("Port2", port2.getVoltage());
//            joinedTelemetry.addData("Port3", port3.getVoltage());
            joinedTelemetry.update();
        }
    }

    public double setRaiserPID(double targetPos) {
        raiserController.setPID(p, i, d);
        currentPos = intake.raiserEnc.getCurrentPosition();
        raiserPower = raiserController.calculate(currentPos, targetPos);
        raiserPower = clamp(raiserPower, -maxPow, maxPow);
        return raiserPower;
    }

    public void setRaiser(double target){
        intake.raiserRight.setPower(setRaiserPID(target));
    }

    //Intake down: 228 goto 222
    //intake up: 194 goto 190

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    //analog0 clutch
    //analog1 turret
    //analog3 intake

    //intake down = 175
    //intake up = 210
}