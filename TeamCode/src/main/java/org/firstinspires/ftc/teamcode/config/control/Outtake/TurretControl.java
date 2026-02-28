package org.firstinspires.ftc.teamcode.config.control.Outtake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretAimController;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;
import org.psilynx.psikit.core.Logger;

public class TurretControl implements Control {
    private static final String LOG_PREFIX = "TurretControl/";

    //---------------- Software ----------------
    Turret turret;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    // Toggle aim lock on GP1 A or GP2 A
    EdgeDetector toggleAimLockStart = new EdgeDetector(() -> turret.toggleAimLock());

    // Relocalize from goal tag on GP2 BACK
    EdgeDetector relocalizeTrigger;

    // Relocalization feedback
    private String relocalizeStatus = "";
    private final ElapsedTime relocalizeStatusTimer = new ElapsedTime();
    private static final double RELOCALIZE_STATUS_DISPLAY_SEC = 3.0;

    //---------------- Constructor ----------------
    public TurretControl(Turret turret, Gamepad gp1, Gamepad gp2) {
        this.turret = turret;
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.relocalizeTrigger = new EdgeDetector(() -> {}); // no-op without robot
    }

    public TurretControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtake.turret, gp1, gp2);
        this.robot = robot;
        this.relocalizeTrigger = new EdgeDetector(() -> doRelocalize());
    }

    //---------------- Methods ----------------

    private void doRelocalize() {
        if (robot == null) return;
        Robot.GoalTagRelocalizeResult result = robot.relocalizeFromGoalTag();
        if (result.success) {
            relocalizeStatus = "OK tag=" + result.tagId;
            if (result.followerPoseBefore != null && result.relocalizedPose != null) {
                double dx = result.relocalizedPose.getX() - result.followerPoseBefore.getX();
                double dy = result.relocalizedPose.getY() - result.followerPoseBefore.getY();
                double dh = Math.toDegrees(result.relocalizedPose.getHeading()
                        - result.followerPoseBefore.getHeading());
                relocalizeStatus += String.format(" d(%.1f,%.1f,%.1f°)", dx, dy, dh);
            }
        } else {
            relocalizeStatus = "FAIL: " + result.reason;
        }
        relocalizeStatusTimer.reset();
        Logger.recordOutput(LOG_PREFIX + "RelocalizeResult", relocalizeStatus);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void update() {
        // Aim lock toggle
        toggleAimLockStart.update(gp1.a || gp2.a);

        // Relocalize trigger
        relocalizeTrigger.update(gp2.back);

        // Manual nudge: GP2 right stick X → turret angular velocity
        // Only active when aim lock is off (MANUAL mode)
        if (!turret.isAimLockEnabled()) {
            double nudgeInput = gp2.right_stick_x;
            // Apply a small deadband
            if (Math.abs(nudgeInput) < 0.05) {
                nudgeInput = 0.0;
            }
            double velocityDegPerSec = nudgeInput * TurretAimController.maxManualVelocityDegPerSec;
            turret.setManualVelocity(velocityDegPerSec);
        } else {
            turret.setManualVelocity(0.0);
        }

        // Log
        Logger.recordOutput(LOG_PREFIX + "AimLockEnabled", turret.isAimLockEnabled());
        Logger.recordOutput(LOG_PREFIX + "ManualNudgeInput", gp2.right_stick_x);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Aim Lock", turret.isAimLockEnabled());
        telemetry.addData("Aim Mode", turret.getAimMode());
        telemetry.addData("Lock Source", turret.getActiveLockSource());
        telemetry.addData("Turret Pos (deg)", "%.1f", turret.getCurrentDegrees());
        telemetry.addData("On Target", turret.isOnTarget());

        // Relocalization status (displayed for a few seconds after triggering)
        if (!relocalizeStatus.isEmpty()
                && relocalizeStatusTimer.seconds() < RELOCALIZE_STATUS_DISPLAY_SEC) {
            telemetry.addData("Relocalize", relocalizeStatus);
        }
    }
}
