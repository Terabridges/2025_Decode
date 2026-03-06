package org.firstinspires.ftc.teamcode.opmodes.tests.OLD;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Disabled
//@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="SpindexTuner", group="Test")
public class SpindexTuner extends OpMode {

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private CRServo spindexLeft;
    private CRServo spindexRight;
    private AnalogInput spindexAnalog;
    private AbsoluteAnalogEncoder spindexEnc;
    private Util util;

    private JoinedTelemetry joinedTelemetry;

    public PIDFController spindexPID;
    public static double p = 0.0, i = 0.0, d = 0.0, f = 0.0;
    public static double posTolerance = 5;
    public static double integrationBounds = 5;
    private double spindexPower = 0.0;
    public static double spindexTarget = 0.0;
    public static double spindexMaxPower = 1.0;
    public static double currentPos = 0;
    private boolean useSpindexPID = true;

    @Override
    public void init() {
        spindexLeft = hardwareMap.get(CRServo.class, "spindexL");
        spindexRight = hardwareMap.get(CRServo.class, "spindexR");
        spindexAnalog = hardwareMap.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 29, 1);
        spindexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexRight.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexPID = new PIDFController(p, i, d, f);
        spindexPID.setIntegrationBounds(-integrationBounds, integrationBounds);
        spindexPID.setTolerance(posTolerance);
        util = new Util();

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

    }

    @Override
    public void loop() {
        gamepadUpdate();
        if (useSpindexPID){
            setSpindex(spindexTarget);
        } else {
            setSpindexPow(0);
        }

        joinedTelemetry.addData("Target Pos", getTargetPos());
        joinedTelemetry.addData("Current Pos", getCurrentPos());
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void setSpindexPow(double pow){
        spindexLeft.setPower(pow);
        spindexRight.setPower(pow);
    }

    public double setSpindexPID(double targetPos) {
        spindexPID.setPIDF(p, i, d, f);
        currentPos = spindexEnc.getCurrentPosition();
        spindexPower = spindexPID.calculate(currentPos, targetPos);
        spindexPower = util.clamp(spindexPower, -spindexMaxPower, spindexMaxPower);
        return spindexPower;
    }

    public void setSpindex(double target){
        setSpindexPow(setSpindexPID(target));
    }

    public double getCurrentPos(){
        return currentPos;
    }

    public double getTargetPos(){
        return spindexTarget;
    }


}
