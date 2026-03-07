package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="IntakeTest", group="Test")
public class IntakeTest extends OpMode {

    //Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    public DcMotor intakeLeft;
    public DcMotorEx intakeRight;
    private Util util;

    String currentMotor = "left";

    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;

    @Override
    public void init() {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        util = new Util();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        gamepadUpdate();
        if(currentGamepad1.a && !previousGamepad1.a){
            if(currentMotor.equals("left")){
                currentMotor = "right";
            } else if(currentMotor.equals("right")){
                currentMotor = "both";
            } else if (currentMotor.equals("both")){
                currentMotor = "left";
            }
        }

        if (currentMotor.equals("left")){
            intakeLeft.setPower(gamepad1.left_stick_y);
        } else if (currentMotor.equals("right")){
            intakeRight.setPower(gamepad1.left_stick_y);
        } else if (currentMotor.equals("both")){
            intakeLeft.setPower(gamepad1.left_stick_y);
            intakeRight.setPower(gamepad1.left_stick_y);
        }

        joinedTelemetry.addData("RPM", velToRPM(intakeRight.getVelocity()));
        joinedTelemetry.addData("Current Motor", currentMotor);
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public double velToRPM(double tps) {
        double rps = tps / (TICKS_PER_REV * SHOOTER_GEAR_RATIO);
        return rps * 60;
    }
}