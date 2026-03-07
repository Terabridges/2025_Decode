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
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "FollowerHeadingTurretLockTest", group = "Test")
public class FollowerHeadingTurretLockTest extends OpMode {

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    public static double manualStepDeg = 2.0;
    public static double manualTurretDeg = 180.0;
    public static double fieldLockDirection = -1.0;

    private boolean fieldLockEnabled = false;
    private double heldFieldHeadingDeg = Double.NaN;

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
    }

    @Override
    public void start() {
        boolean reuseAutoFollower = GlobalVariables.isAutoFollowerValid()
                && FollowerManager.follower != null;
        if (reuseAutoFollower) {
            FollowerManager.getFollower(hardwareMap);
        } else {
            double allianceHeading = GlobalVariables.isBlueAlliance() ? Math.PI : 0.0;
            FollowerManager.initFollower(hardwareMap, new Pose(72, 72, allianceHeading));
        }
        GlobalVariables.setAutoFollowerValid(false);

        manualTurretDeg = robot.outtake.turret.getCurrentDegrees();
        disableIntake();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (FollowerManager.follower != null) {
            FollowerManager.follower.update();
        }

        disableIntake();
        robot.update();
        disableIntake();

        Pose pose = (FollowerManager.follower != null) ? FollowerManager.follower.getPose() : null;

        if (edge(currentGamepad1.y, previousGamepad1.y)) {
            captureAndEnableFieldHeadingLock(pose);
        }
        if (edge(currentGamepad1.b, previousGamepad1.b)) {
            fieldLockEnabled = false;
        }

        if (edge(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) {
            manualTurretDeg = normalizeDeg(manualTurretDeg + manualStepDeg);
            if (!fieldLockEnabled) {
                robot.outtake.turret.setTurretDegree(manualTurretDeg);
            }
        }
        if (edge(currentGamepad1.dpad_left, previousGamepad1.dpad_left)) {
            manualTurretDeg = normalizeDeg(manualTurretDeg - manualStepDeg);
            if (!fieldLockEnabled) {
                robot.outtake.turret.setTurretDegree(manualTurretDeg);
            }
        }
        if (edge(currentGamepad1.a, previousGamepad1.a)) {
            fieldLockEnabled = false;
            robot.outtake.turret.setTurretDegree(manualTurretDeg);
        }

        double robotHeadingDeg = Double.NaN;
        double targetTurretDeg = Double.NaN;
        if (pose != null) {
            robotHeadingDeg = normalizeDeg(Math.toDegrees(pose.getHeading()));
        }

        if (fieldLockEnabled && !Double.isNaN(robotHeadingDeg)) {
            double errorDeg = wrapSignedDeg(heldFieldHeadingDeg - robotHeadingDeg);
            targetTurretDeg = normalizeDeg(Turret.turretForwardDeg + (fieldLockDirection * errorDeg));
            robot.outtake.turret.setTurretDegree(targetTurretDeg);
        }

        // POV mecanum drive so heading can change while testing lock behavior.
        double axial = -currentGamepad1.left_stick_y;
        double lateral = currentGamepad1.left_stick_x;
        double yaw = currentGamepad1.right_stick_x;
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        robot.other.drive.setDrivePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        joinedTelemetry.addData("Field Lock", fieldLockEnabled);
        joinedTelemetry.addData("Held Field Heading (deg)", fmt(heldFieldHeadingDeg));
        joinedTelemetry.addData("Robot Heading (deg)", fmt(robotHeadingDeg));
        joinedTelemetry.addData("Target Turret (deg)", fmt(targetTurretDeg));
        joinedTelemetry.addData("Current Turret (deg)", "%.1f", robot.outtake.turret.getCurrentDegrees());
        joinedTelemetry.addData("Manual Turret (deg)", "%.1f", manualTurretDeg);
        joinedTelemetry.addData("Lock Direction", "%.1f", fieldLockDirection);
        joinedTelemetry.addData("Controls", "Y capture+lock, B unlock, A manual set, Dpad L/R nudge");
        joinedTelemetry.update();
    }

    private void disableIntake() {
        robot.intake.autoIntake = false;
        robot.intake.spinner.autoSpin = false;
        robot.intake.spinner.setMegaSpinZero();
    }

    private void captureAndEnableFieldHeadingLock(Pose pose) {
        if (pose == null) {
            fieldLockEnabled = false;
            heldFieldHeadingDeg = Double.NaN;
            return;
        }

        double robotHeadingDeg = normalizeDeg(Math.toDegrees(pose.getHeading()));
        double turretDeg = normalizeDeg(robot.outtake.turret.getCurrentDegrees());
        double robotRelativeTurretDeg = wrapSignedDeg(turretDeg - Turret.turretForwardDeg);
        heldFieldHeadingDeg = normalizeDeg(robotHeadingDeg + robotRelativeTurretDeg);
        fieldLockEnabled = true;
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

    private static String fmt(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "n/a";
        return String.format("%.1f", v);
    }
}

