package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;
import org.psilynx.psikit.ftc.FtcLogTuning;
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name="TurretTest", group="Test")
public class TurretTest extends OpMode {

    Robot robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    private JoinedTelemetry joinedTelemetry;

    public static double degree = 0;
    public static double defaultStartX = 72.0;
    public static double defaultStartY = 72.0;
    public static double defaultStartHeadingDeg = 0.0;
    public static double fieldLockDirection = -1.0; // set to -1.0 if lock moves opposite of expected

    private boolean fieldHeadingLockEnabled = false;
    private double heldFieldHeadingDeg = Double.NaN;

    private void configureLowOverheadPsiKitLogging() {
        FtcLogTuning.nonBulkReadPeriodSec = 0.10;
        FtcLogTuning.processColorDistanceSensorsInBackground = false;
        FtcLogTuning.pinpointLoggerCallsUpdate = false;
        FtcLogTuning.pinpointReadPeriodSec = .10;
    }

    @Override
    public void init() {
        configureLowOverheadPsiKitLogging();
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        FollowerManager.getFollower(
            hardwareMap,
            new Pose(defaultStartX, defaultStartY, Math.toRadians(defaultStartHeadingDeg))
        );

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
        //robot.toInit();
        robot.outtake.turret.toInit();
    }

    @Override
    public void loop() {
        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }
        robot.outtake.turret.update();
        gamepadUpdate();
        if(currentGamepad1.a && !previousGamepad1.a){
            robot.outtake.turret.setTurretDegree(degree);
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            captureAndEnableFieldHeadingLock();
        }
        if (currentGamepad1.b && !previousGamepad1.b) {
            fieldHeadingLockEnabled = false;
        }

        if (fieldHeadingLockEnabled && FollowerManager.follower != null && FollowerManager.follower.getPose() != null) {
            double robotHeadingDeg = Math.toDegrees(FollowerManager.follower.getPose().getHeading());
            double relativeDeg = fieldLockDirection * wrapSignedDeg(heldFieldHeadingDeg - robotHeadingDeg);
            double targetTurretDeg = normalizeDeg(Turret.turretForwardDeg + relativeDeg);
            robot.outtake.turret.setTurretDegree(targetTurretDeg);
        }

        double robotHeadingDeg = Double.NaN;
        if (FollowerManager.follower != null && FollowerManager.follower.getPose() != null) {
            robotHeadingDeg = Math.toDegrees(FollowerManager.follower.getPose().getHeading());
        }
        joinedTelemetry.addData("Set Degree (A)", degree);
        joinedTelemetry.addData("Turret Deg", robot.outtake.turret.getCurrentDegrees());
        joinedTelemetry.addData("Robot Heading Deg", fmt(robotHeadingDeg));
        joinedTelemetry.addData("Field Lock Enabled", fieldHeadingLockEnabled);
        joinedTelemetry.addData("Field Lock Direction", fieldLockDirection);
        joinedTelemetry.addData("Held Field Heading Deg", fmt(heldFieldHeadingDeg));
        joinedTelemetry.addData("Controls", "A=set degree, Y=capture+enable field lock, B=disable lock");

        joinedTelemetry.update();
    }

    private void captureAndEnableFieldHeadingLock() {
        if (FollowerManager.follower == null || FollowerManager.follower.getPose() == null) {
            fieldHeadingLockEnabled = false;
            return;
        }
        double robotHeadingDeg = Math.toDegrees(FollowerManager.follower.getPose().getHeading());
        double turretDeg = robot.outtake.turret.getCurrentDegrees();
        double turretRelativeDeg = fieldLockDirection * wrapSignedDeg(turretDeg - Turret.turretForwardDeg);
        heldFieldHeadingDeg = normalizeDeg(robotHeadingDeg + turretRelativeDeg);
        fieldHeadingLockEnabled = true;
    }

    private static double normalizeDeg(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    private static double wrapSignedDeg(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static String fmt(double v) {
        if (Double.isNaN(v)) return "n/a";
        return String.format("%.2f", v);
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}
