package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="TurretTest", group="Test")
public class TurretTest extends OpMode {

    //Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    public static double leftDegree = 0;
    public static double rightDegree = 0;
    public static double degree = 0;
    public static double rightOff = 0.031;

    private Servo leftTurret;
    private Servo rightTurret;
    private AnalogInput turretAnalog;
    private AbsoluteAnalogEncoder turretEnc;
    private Util util;

    @Override
    public void init() {
        //robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        util = new Util();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        leftTurret = hardwareMap.get(Servo.class, "turretL");
        rightTurret = hardwareMap.get(Servo.class, "turretR");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        turretEnc.setInverted(true);
        leftTurret.setDirection(Servo.Direction.FORWARD);
        rightTurret.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        gamepadUpdate();
        rightDegree = util.clamp(rightDegree/360, 0, 1-rightOff);
        leftDegree = util.clamp(leftDegree/360, 0, 1-rightOff);
        degree = util.clamp(degree/360, 0, 1-rightOff);
        if(currentGamepad1.b && !previousGamepad1.b){
            rightTurret.setPosition(rightDegree+rightOff);
        }

        if(currentGamepad1.x && !previousGamepad1.x){
            leftTurret.setPosition(leftDegree);
        }

        if(currentGamepad1.y && !previousGamepad1.y){
            leftTurret.setPosition(degree);
            rightTurret.setPosition(degree+rightOff);
        }

        joinedTelemetry.addData("Position", turretEnc.getCurrentPosition());
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}