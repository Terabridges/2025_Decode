package org.firstinspires.ftc.teamcode.opmodes.tests.OLD;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
@TeleOp(name="TurretTuner", group="Test")
public class TurretTuner extends OpMode {

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private CRServo leftTurret;
    private CRServo rightTurret;
    private AnalogInput turretAnalog;
    private AbsoluteAnalogEncoder turretEnc;
    private Util util;

    private JoinedTelemetry joinedTelemetry;

    public PIDController turretPID;
    public static double p = 0.01, i = 0.0, d = 0.0005; //feedforward seems to only be adding not subtracting 0.0004
    public static double posTolerance = 5;
    public static double integrationBounds = 10;
    private double turretPower = 0.0;
    public static double turretTarget = 0.0;
    public static double turretMaxPower = 1.0;
    public static double currentPos = 0;
    private boolean useTurretPID = true;

    public SimpleMotorFeedforward feedforward;
    public static double ks = 0.0, kv = 0.0, ka = 0.0;


    private double turretCounterClockwiseLimit = 170;
    private double turretClowckwiseLimit = 60;

    private double TurretForward = 210;

    @Override
    public void init() {
        leftTurret = hardwareMap.get(CRServo.class, "turretL");
        rightTurret = hardwareMap.get(CRServo.class, "turretR");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        leftTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretPID = new PIDController(p, i, d);
        turretPID.setIntegrationBounds(-integrationBounds, integrationBounds);
        turretPID.setTolerance(posTolerance);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
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
        if (useTurretPID){
            setTurret(turretTarget);
        } else {
            setTurretPow(0);
        }

        joinedTelemetry.addData("Current Pos", getCurrentPos());
        joinedTelemetry.addData("Target Pos", getTargetPos());
        joinedTelemetry.addData("Current Pow", turretPower);
        joinedTelemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void setTurretPow(double pow){
        leftTurret.setPower(pow);
        rightTurret.setPower(pow);
    }

    public double setTurretPID(double targetPos) {
        turretPID.setPID(p, i, d);
        currentPos = turretEnc.getCurrentPosition();
        turretPower = turretPID.calculate(currentPos, targetPos);
        turretPower = util.clamp(turretPower, -turretMaxPower, turretMaxPower);
        return turretPower;
    }

    public void setTurret(double target){
        setTurretPow(setTurretPID(target));
    }

    public double getCurrentPos(){
        return currentPos;
    }

    public double getTargetPos(){
        return turretTarget;
    }

}
