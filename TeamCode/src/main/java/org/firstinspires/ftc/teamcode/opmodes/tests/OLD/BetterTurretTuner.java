package org.firstinspires.ftc.teamcode.opmodes.tests.OLD;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
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
@TeleOp(name="BetterTurretTuner", group="Test")
public class BetterTurretTuner extends OpMode {

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

    private TrapezoidProfile.Constraints constraints;
    private ProfiledPIDController turretController;
    private SimpleMotorFeedforward turretFeedforward;
    public static double maxVel = 180, maxAcc = 900, maxPow = 1.0;
    public static double p = 0.0, i = 0.0, d = 0.0;
    public static double ks = 0.0, kv = 0.0, ka = 0.0;

    private double minDegrees = 60, maxDegrees = 270;

    public static double turretTarget = 0.0;
    private double turretPower = 0.0;
    private double currentPos = 0;
    private boolean useTurretPID = true;


    private double TurretForward = 210;

    @Override
    public void init() {
        leftTurret = hardwareMap.get(CRServo.class, "turretL");
        rightTurret = hardwareMap.get(CRServo.class, "turretR");
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        leftTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        util = new Util();
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);
        turretController = new ProfiledPIDController(p, i, d, constraints);
        turretFeedforward = new SimpleMotorFeedforward(ks, kv, ka);
    }

    @Override
    public void start(){
        turretController.reset(getCurrentPos());
    }

    @Override
    public void loop() {
        gamepadUpdate();
        if (useTurretPID){
            //setTurret(turretTarget);
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

    public double getCurrentPos(){
        return turretEnc.getCurrentPosition();
    }

    public double getTargetPos(){
        return turretTarget;
    }

//    public double updateTurretController(double currentDeg, double currentVel, double targetDeg, double loopTime) {
//        targetDeg = util.clamp(targetDeg, minDegrees, maxDegrees);
//        turretController.setGoal(targetDeg);
//        double pidOutput = turretController.calculate(currentDeg);
//
//        TrapezoidProfile.State sp = turretController.getSetpoint();
//        double velDes = sp.velocity;
//    }

}
