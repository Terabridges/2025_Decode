package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="SpindexTest", group="Test")
public class NewSpindexTest extends OpMode {

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    private JoinedTelemetry joinedTelemetry;
    private Spindex spindex;

    public static double degree = 90;


    @Override
    public void init() {
        spindex = new Spindex(hardwareMap);

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        gamepadUpdate();
        if (currentGamepad1.a && !previousGamepad1.a){
            spindex.setSpindexDegree(degree);
        }

        joinedTelemetry.addData("Current/Commanded Pos", String.format("%.1f",spindex.getAbsolutePos()) + "/" + String.format("%.1f",spindex.getCommandedPos()));
        joinedTelemetry.addData("Commanded Pos", spindex.getCommandedDegree());
        joinedTelemetry.addData("Absolute Pos", spindex.getAbsolutePos());
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
    }

}