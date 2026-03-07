package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="SpindexTest", group="Test")
public class SpindexTest extends OpMode {

    //Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    public static double leftDegree = 0;
    public static double rightDegree = 0;
    public static double degree = 0;

    private Servo spindexLeft;
    private Servo spindexRight;
    private AnalogInput spindexAnalog;
    private AbsoluteAnalogEncoder spindexEnc;
    //private Intake intake;

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

        spindexLeft = hardwareMap.get(Servo.class, "spindexL");
        spindexRight = hardwareMap.get(Servo.class, "spindexR");
        spindexAnalog = hardwareMap.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 29, 1.17);
        spindexEnc.setInverted(false);
        spindexRight.setDirection(Servo.Direction.FORWARD);
        spindexLeft.setDirection(Servo.Direction.FORWARD);
        //intake = new Intake(hardwareMap);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        gamepadUpdate();
        if(currentGamepad1.b && !previousGamepad1.b){
            spindexRight.setPosition(rightDegree/360);
        }

        if(currentGamepad1.x && !previousGamepad1.x){
            spindexLeft.setPosition(leftDegree/360);
        }

        if(currentGamepad1.y && !previousGamepad1.y){
            spindexRight.setPosition(degree/360);
            spindexLeft.setPosition(degree/360);
        }

        joinedTelemetry.addData("RawEncoderVolts", spindexAnalog.getVoltage());
        joinedTelemetry.addData("Position", spindexEnc.getCurrentPosition());
        joinedTelemetry.update();

        Logger.recordOutput("SpindexDeg", spindexEnc.getCurrentPosition());
        //Logger.recordOutput("RobotAmps", intake.getFloodgateCurrentAmps() );
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}