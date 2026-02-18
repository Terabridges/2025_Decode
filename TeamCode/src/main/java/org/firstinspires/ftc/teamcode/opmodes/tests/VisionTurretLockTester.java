package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@Configurable
//@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "VisionTurretLockTester", group = "Test")
public class VisionTurretLockTester extends OpMode {

    private Robot robot;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private JoinedTelemetry joinedTelemetry;

    public static int requiredTagId = 20;
    public static boolean lockEnabled = true;
    public static double manualStepDeg = 2.0;
    public static boolean enableLimitChassisAssist = true;
    public static double chassisYawKp = 0.02;
    public static double chassisYawMax = 0.25;
    public static double chassisYawDirection = 1.0; // set to -1.0 to invert assist direction
    public static double turretVisionDirection = -1.0; // tester-only direction trim for tx lock

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        robot.toInit();
        robot.other.drive.manualDrive = true;
        robot.outtake.vision.setRequiredTagId(requiredTagId);
        Turret.visionDirection = turretVisionDirection;
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        robot.update();
        robot.outtake.vision.setRequiredTagId(requiredTagId);
        Turret.visionDirection = turretVisionDirection;

        if (currentGamepad1.a && !previousGamepad1.a) {
            lockEnabled = !lockEnabled;
        }
        if (currentGamepad1.x && !previousGamepad1.x) {
            requiredTagId = 20;
        }
        if (currentGamepad1.y && !previousGamepad1.y) {
            requiredTagId = 24;
        }
        if (currentGamepad1.b && !previousGamepad1.b) {
            robot.outtake.turret.toInit();
        }
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            robot.outtake.turret.setTurretDegree(robot.outtake.turret.getCurrentDegrees() - manualStepDeg);
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            robot.outtake.turret.setTurretDegree(robot.outtake.turret.getCurrentDegrees() + manualStepDeg);
        }

        boolean seesRequired = robot.outtake.vision.hasRequiredTarget();
        double tx = robot.outtake.vision.getTxForTag(requiredTagId);
        double distanceIn = robot.outtake.vision.getDistanceInchesForTag(requiredTagId);
        double yawAssist = 0.0;
        boolean assistActive = false;

        if (lockEnabled && seesRequired) {
            robot.outtake.turret.aimFromVision(tx, distanceIn);
            if (enableLimitChassisAssist && robot.outtake.turret.needsChassisYawAssist(tx, distanceIn)) {
                assistActive = true;
                yawAssist = tx * chassisYawKp * chassisYawDirection;
                yawAssist = Math.max(-chassisYawMax, Math.min(chassisYawMax, yawAssist));
            }
        }

        // POV mecanum drive: left stick = translation, right stick x = rotation.
        double axial = -currentGamepad1.left_stick_y;
        double lateral = currentGamepad1.left_stick_x;
        double yaw = currentGamepad1.right_stick_x + yawAssist;
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

        joinedTelemetry.addData("Lock Enabled (A)", lockEnabled);
        joinedTelemetry.addData("Required Tag (X=20, Y=24)", requiredTagId);
        joinedTelemetry.addData("Sees Required", seesRequired);
        joinedTelemetry.addData("Tx (deg)", tx);
        joinedTelemetry.addData("Distance (in)", distanceIn);
        joinedTelemetry.addData("Turret Deg", robot.outtake.turret.getCurrentDegrees());
        joinedTelemetry.addData("Limit Assist Enabled", enableLimitChassisAssist);
        joinedTelemetry.addData("Limit Assist Active", assistActive);
        joinedTelemetry.addData("Yaw Assist", yawAssist);
        joinedTelemetry.addData("Turret Vision Dir", Turret.visionDirection);
        joinedTelemetry.addData("Manual Nudge Step", manualStepDeg);
        joinedTelemetry.addData("Controls", "A toggle lock, X/Y tag, B reset turret, Dpad L/R nudge");
        joinedTelemetry.update();
    }
}
