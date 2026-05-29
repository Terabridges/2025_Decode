package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
//@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="HoodAdjust", group="Test")
public class HoodAdjust extends OpMode {

    //Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;
    private Servo hood;
    double hoodUp = 1.0;
    double hoodDown = 0.43;


    @Override
    public void init() {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        hood = hardwareMap.get(Servo.class, "hood");

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        gamepadUpdate();
        if(currentGamepad1.y && !previousGamepad1.y){
            hood.setPosition(hoodUp);
        }

        if(currentGamepad1.a && !previousGamepad1.a){
            hood.setPosition(hoodDown);
        }

        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}
