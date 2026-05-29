package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
//@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="KickerTest", group="Test")
public class KickerTest extends OpMode {

    //Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    private CRServo kickerLeft;
    private CRServo kickerRight;
    private AnalogInput kickerAnalog;
    private AbsoluteAnalogEncoder kickerEnc;

    String currentServo = "left";

    @Override
    public void init() {
        //robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        kickerLeft = hardwareMap.get(CRServo.class, "kickL");
        kickerRight = hardwareMap.get(CRServo.class, "kickR");
        kickerAnalog = hardwareMap.get(AnalogInput.class, "kickAnalog");
        kickerEnc = new AbsoluteAnalogEncoder(kickerAnalog, 3.3, 80, 1);
        kickerLeft.setDirection(CRServo.Direction.FORWARD);
        kickerRight.setDirection(CRServo.Direction.REVERSE);

    }

    //355 down, goes down until 10

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        gamepadUpdate();
        if(currentGamepad1.a && !previousGamepad1.a){
            if(currentServo.equals("left")){
                currentServo = "right";
            } else if(currentServo.equals("right")){
                currentServo = "both";
            } else if (currentServo.equals("both")){
                currentServo = "left";
            }
        }

        if (currentServo.equals("left")){
            kickerLeft.setPower(gamepad1.left_stick_y);
        } else if (currentServo.equals("right")){
            kickerRight.setPower(gamepad1.left_stick_y);
        } else if (currentServo.equals("both")){
            kickerLeft.setPower(gamepad1.left_stick_y);
            kickerRight.setPower(gamepad1.left_stick_y);
        }

        joinedTelemetry.addData("Current Servo", currentServo);
        joinedTelemetry.addData("Pos", kickerEnc.getCurrentPosition());
        joinedTelemetry.update();

        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}
