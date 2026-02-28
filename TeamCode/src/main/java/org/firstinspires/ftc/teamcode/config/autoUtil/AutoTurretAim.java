package org.firstinspires.ftc.teamcode.config.autoUtil;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.TurretAimController;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

public class AutoTurretAim {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double BLUE_VISION_DIRECTION = -1.0;
    private static final double RED_VISION_DIRECTION = -1.0;

    private final Robot robot;
    private final Alliance alliance;
    private final Telemetry telemetry;

    public AutoTurretAim(Robot robot, AutoPoses poses, Alliance alliance, Range range, Telemetry telemetry) {
        this.robot = robot;
        this.alliance = alliance;
        this.telemetry = telemetry;
    }

    public void updateAim(AutoStates activeState, boolean preloadComplete) {
        if (robot == null || robot.outtake == null || robot.outtake.turret == null || robot.outtake.vision == null) {
            return;
        }

        int requiredGoalTagId = (alliance == Alliance.BLUE) ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
        robot.outtake.vision.setRequiredTagId(requiredGoalTagId);
        TurretAimController.cameraLateralOffsetIn = 0.0;
        TurretAimController.visionDirection = (alliance == Alliance.BLUE) ? BLUE_VISION_DIRECTION : RED_VISION_DIRECTION;

        boolean acquireMotif = activeState == AutoStates.ACQUIRE_MOTIF;
        if (activeState == AutoStates.ACQUIRE_MOTIF) {
            // Acquire motif uses only direct ODO obelisk aim (no aimLock).
            if (robot.outtake.turret.isAimLockEnabled()) {
                robot.outtake.turret.setAimLockEnabled(false);
            }
            robot.outtake.turret.setAimTargetObelisk();
            robot.outtake.turret.aimAtObeliskWithOdometry();
        } else {
            // All non-acquire states use the same aimLock flow as teleop.
            if (!robot.outtake.turret.isAimLockEnabled()) {
                robot.outtake.turret.setAimLockEnabled(true);
            }
            robot.outtake.turret.setAimTargetGoal();
        }

        telemetry.addData("Auto Acquire Motif", acquireMotif);
        telemetry.addData("Auto Aim Lock", robot.outtake.turret.isAimLockEnabled());
        telemetry.addData("Auto Aim Source", robot.outtake.turret.getActiveLockSource());
        telemetry.addData("Auto Aim Target", robot.outtake.turret.getAimTarget());
        double commandedDeg = robot.outtake.turret.getCurrentDegrees();
        double mappedEncoderDeg = robot.outtake.turret.getMappedEncoderTurretDegrees();
        telemetry.addData("Turret Cmd (deg)", "%.2f", commandedDeg);
        if (Double.isNaN(mappedEncoderDeg) || Double.isInfinite(mappedEncoderDeg)) {
            telemetry.addData("Turret Mapped Enc (deg)", "N/A");
            telemetry.addData("Turret Cmd-Map Delta (deg)", "N/A");
        } else {
            double cmdMinusMapDeltaDeg = ((commandedDeg - mappedEncoderDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
            telemetry.addData("Turret Mapped Enc (deg)", "%.2f", mappedEncoderDeg);
            telemetry.addData("Turret Cmd-Map Delta (deg)", "%.2f", cmdMinusMapDeltaDeg);
        }
        telemetry.addData("Required Goal Tag Id", requiredGoalTagId);
    }
}
