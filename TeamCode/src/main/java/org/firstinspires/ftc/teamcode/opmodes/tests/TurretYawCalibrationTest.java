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
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
//@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretYawCalibrationTest", group = "Test")
public class TurretYawCalibrationTest extends OpMode {

    public static double startXIn = 72.0;
    public static double startYIn = 72.0;
    public static double startHeadingDeg = 0.0;
    public static boolean useAllianceDefaultHeading = true;

    public static double manualStepDeg = 1.0;
    public static double manualFastStepDeg = 5.0;
    public static double forwardErrorWarnDeg = 5.0;
    public static double cmdVsMappedWarnDeg = 8.0;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();

    private double commandedTargetDeg = 180.0;

    private double captureForwardCmdDeg = Double.NaN;
    private double captureForwardEncDeg = Double.NaN;
    private double captureForwardMappedDeg = Double.NaN;
    private double captureMinDeg = Double.NaN;
    private double captureMaxDeg = Double.NaN;
    private String lastCapture = "none";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        robot.toInit();
        robot.other.drive.manualDrive = true;
        robot.outtake.setAimLockEnabled(false);
        disableIntake();

        commandedTargetDeg = robot.outtake.turret.getCurrentDegrees();
    }

    @Override
    public void start() {
        boolean reuseAutoFollower = GlobalVariables.isAutoFollowerValid()
                && FollowerManager.follower != null;
        if (reuseAutoFollower) {
            FollowerManager.getFollower(hardwareMap);
        } else {
            double headingRad = Math.toRadians(startHeadingDeg);
            if (useAllianceDefaultHeading) {
                headingRad = GlobalVariables.isBlueAlliance() ? Math.PI : 0.0;
            }
            FollowerManager.initFollower(hardwareMap, new Pose(startXIn, startYIn, headingRad));
        }
        GlobalVariables.setAutoFollowerValid(false);

        robot.outtake.turret.syncCommandToMeasured();
        commandedTargetDeg = robot.outtake.turret.getCurrentDegrees();
    }

    @Override
    public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }

        robot.outtake.setAimLockEnabled(false);
        disableIntake();

        handleTurretControls();

        robot.update();

        double commandedDeg = normalizeDeg(robot.outtake.turret.getCurrentDegrees());
        double encoderDeg = normalizeDeg(robot.outtake.turret.getEncoderDegrees());
        double mappedDeg = normalizeDeg(robot.outtake.turret.getMappedEncoderTurretDegrees());
        double mappedVsCmdDeg = wrapSignedDeg(mappedDeg - commandedDeg);
        double commandedRelativeDeg = wrapSignedDeg(commandedDeg - Turret.turretForwardDeg);
        double mappedRelativeDeg = wrapSignedDeg(mappedDeg - Turret.turretForwardDeg);
        boolean increasingCmd = wrapSignedDeg(commandedDeg - commandedTargetDeg) >= 0.0;

        double suggestedForwardDeg = captureForwardMappedDeg;
        double suggestedMinDeg = Double.NaN;
        double suggestedMaxDeg = Double.NaN;
        if (!Double.isNaN(captureMinDeg) && !Double.isNaN(captureMaxDeg)) {
            suggestedMinDeg = Math.min(captureMinDeg, captureMaxDeg);
            suggestedMaxDeg = Math.max(captureMinDeg, captureMaxDeg);
        }

        double chassisYawDeg = robot.outtake.vision.getLastChassisYawDeg();
        double visionTurretRelDeg = robot.outtake.vision.getLastTurretRelativeYawDeg();
        double visionYawSentDeg = robot.outtake.vision.getLastRobotYawSentDeg();

        Logger.recordOutput("TurretCal/CmdDeg", commandedDeg);
        Logger.recordOutput("TurretCal/TargetCmdDeg", commandedTargetDeg);
        Logger.recordOutput("TurretCal/EncoderRawDeg", encoderDeg);
        Logger.recordOutput("TurretCal/EncoderMappedDeg", mappedDeg);
        Logger.recordOutput("TurretCal/MappedMinusCmdDeg", mappedVsCmdDeg);
        Logger.recordOutput("TurretCal/ForwardRefDeg", Turret.turretForwardDeg);
        Logger.recordOutput("TurretCal/CmdRelativeDeg", commandedRelativeDeg);
        Logger.recordOutput("TurretCal/MappedRelativeDeg", mappedRelativeDeg);
        Logger.recordOutput("TurretCal/CaptureForwardCmdDeg", captureForwardCmdDeg);
        Logger.recordOutput("TurretCal/CaptureForwardEncDeg", captureForwardEncDeg);
        Logger.recordOutput("TurretCal/CaptureForwardMappedDeg", captureForwardMappedDeg);
        Logger.recordOutput("TurretCal/SuggestedForwardDeg", suggestedForwardDeg);
        Logger.recordOutput("TurretCal/CaptureMinDeg", captureMinDeg);
        Logger.recordOutput("TurretCal/CaptureMaxDeg", captureMaxDeg);
        Logger.recordOutput("TurretCal/SuggestedMinDeg", suggestedMinDeg);
        Logger.recordOutput("TurretCal/SuggestedMaxDeg", suggestedMaxDeg);
        Logger.recordOutput("TurretCal/IsCmdIncreaseDirection", increasingCmd ? 1.0 : 0.0);
        Logger.recordOutput("TurretCal/Vision/ChassisYawDeg", chassisYawDeg);
        Logger.recordOutput("TurretCal/Vision/TurretRelativeYawDeg", visionTurretRelDeg);
        Logger.recordOutput("TurretCal/Vision/RobotYawSentDeg", visionYawSentDeg);
        Logger.recordOutput("TurretCal/Vision/RobotYawSendSuccess", robot.outtake.vision.wasLastRobotYawSendSuccessful() ? 1.0 : 0.0);

        joinedTelemetry.addData("Turret cmd", "%.1f", commandedDeg);
        joinedTelemetry.addData("Turret target", "%.1f", commandedTargetDeg);
        joinedTelemetry.addData("Turret enc raw", "%.1f", encoderDeg);
        joinedTelemetry.addData("Turret enc mapped", "%.1f", mappedDeg);
        joinedTelemetry.addData("Mapped-Cmd (deg)", "%.1f", mappedVsCmdDeg);
        joinedTelemetry.addData("Forward ref (deg)", "%.1f", Turret.turretForwardDeg);
        joinedTelemetry.addData("Cmd relative (deg)", "%.1f", commandedRelativeDeg);
        joinedTelemetry.addData("Mapped relative (deg)", "%.1f", mappedRelativeDeg);

        joinedTelemetry.addData("Forward capture cmd", fmt(captureForwardCmdDeg));
        joinedTelemetry.addData("Forward capture enc", fmt(captureForwardEncDeg));
        joinedTelemetry.addData("Forward capture mapped", fmt(captureForwardMappedDeg));
        joinedTelemetry.addData("Suggested turretForwardDeg", fmt(suggestedForwardDeg));
        joinedTelemetry.addData("Capture min", fmt(captureMinDeg));
        joinedTelemetry.addData("Capture max", fmt(captureMaxDeg));
        joinedTelemetry.addData("Suggested turretMinDeg", fmt(suggestedMinDeg));
        joinedTelemetry.addData("Suggested turretMaxDeg", fmt(suggestedMaxDeg));
        joinedTelemetry.addData("Last capture", lastCapture);

        joinedTelemetry.addData("Vision chassis yaw", fmt(chassisYawDeg));
        joinedTelemetry.addData("Vision turret rel yaw", fmt(visionTurretRelDeg));
        joinedTelemetry.addData("Vision yaw sent", fmt(visionYawSentDeg));
        joinedTelemetry.addData("Vision send ok", robot.outtake.vision.wasLastRobotYawSendSuccessful());

        joinedTelemetry.addData("Warn forward err", Math.abs(mappedRelativeDeg) > forwardErrorWarnDeg);
        joinedTelemetry.addData("Warn map-cmd err", Math.abs(mappedVsCmdDeg) > cmdVsMappedWarnDeg);
        joinedTelemetry.addData("Controls", "Dpad=±1 LB/RB=±5 A=cap FWD X=cap MIN Y=cap MAX B=sync");
        joinedTelemetry.update();
    }

    private void handleTurretControls() {
        if (edge(currentGamepad.dpad_right, previousGamepad.dpad_right)) {
            commandedTargetDeg = normalizeDeg(commandedTargetDeg + manualStepDeg);
            robot.outtake.turret.setTurretDegree(commandedTargetDeg);
        }
        if (edge(currentGamepad.dpad_left, previousGamepad.dpad_left)) {
            commandedTargetDeg = normalizeDeg(commandedTargetDeg - manualStepDeg);
            robot.outtake.turret.setTurretDegree(commandedTargetDeg);
        }
        if (edge(currentGamepad.right_bumper, previousGamepad.right_bumper)) {
            commandedTargetDeg = normalizeDeg(commandedTargetDeg + manualFastStepDeg);
            robot.outtake.turret.setTurretDegree(commandedTargetDeg);
        }
        if (edge(currentGamepad.left_bumper, previousGamepad.left_bumper)) {
            commandedTargetDeg = normalizeDeg(commandedTargetDeg - manualFastStepDeg);
            robot.outtake.turret.setTurretDegree(commandedTargetDeg);
        }
        if (edge(currentGamepad.b, previousGamepad.b)) {
            robot.outtake.turret.syncCommandToMeasured();
            commandedTargetDeg = robot.outtake.turret.getCurrentDegrees();
            lastCapture = "B: sync command to encoder";
        }

        if (edge(currentGamepad.a, previousGamepad.a)) {
            captureForwardCmdDeg = normalizeDeg(robot.outtake.turret.getCurrentDegrees());
            captureForwardEncDeg = normalizeDeg(robot.outtake.turret.getEncoderDegrees());
            captureForwardMappedDeg = normalizeDeg(robot.outtake.turret.getMappedEncoderTurretDegrees());
            lastCapture = "A: capture mechanical forward";
        }
        if (edge(currentGamepad.x, previousGamepad.x)) {
            captureMinDeg = normalizeDeg(robot.outtake.turret.getMappedEncoderTurretDegrees());
            lastCapture = "X: capture mapped min";
        }
        if (edge(currentGamepad.y, previousGamepad.y)) {
            captureMaxDeg = normalizeDeg(robot.outtake.turret.getMappedEncoderTurretDegrees());
            lastCapture = "Y: capture mapped max";
        }
    }

    private void disableIntake() {
        robot.intake.autoIntake = false;
        robot.intake.spinner.autoSpin = false;
        robot.intake.spinner.setMegaSpinZero();
    }

    private static boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private static double normalizeDeg(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    private static double wrapSignedDeg(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static String fmt(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return "n/a";
        }
        return String.format("%.1f", value);
    }
}
