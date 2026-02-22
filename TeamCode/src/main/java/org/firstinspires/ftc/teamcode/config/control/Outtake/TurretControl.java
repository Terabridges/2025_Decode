package org.firstinspires.ftc.teamcode.config.control.Outtake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class TurretControl implements Control {

    //---------------- Software ----------------
    Turret turret;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector toggleAimLockStart = new EdgeDetector(() -> turret.toggleAimLock());

    //---------------- Constructor ----------------
    public TurretControl(Turret turret, Gamepad gp1, Gamepad gp2){
        this.turret = turret;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public TurretControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtake.turret, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        toggleAimLockStart.update(gp1.a || gp2.a);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Aim Lock", turret.isAimLockEnabled());
        telemetry.addData("Aim Source", turret.getActiveLockSource());
        telemetry.addData("Aim Target", turret.getAimTarget());
        double commandedDeg = turret.getCurrentDegrees();
        double mappedEncoderDeg = turret.getMappedEncoderTurretDegrees();
        telemetry.addData("Turret Cmd (deg)", "%.2f", commandedDeg);
        if (Double.isNaN(mappedEncoderDeg) || Double.isInfinite(mappedEncoderDeg)) {
            telemetry.addData("Turret Mapped Enc (deg)", "N/A");
            telemetry.addData("Turret Cmd-Map Delta (deg)", "N/A");
        } else {
            double cmdMinusMapDeltaDeg = ((commandedDeg - mappedEncoderDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
            telemetry.addData("Turret Mapped Enc (deg)", "%.2f", mappedEncoderDeg);
            telemetry.addData("Turret Cmd-Map Delta (deg)", "%.2f", cmdMinusMapDeltaDeg);
        }
        telemetry.addData("Launch Zone", turret.isInLaunchZone());
        telemetry.addData("Turret Wrap", turret.isTurretWrapEnabled());
        if (robot != null && robot.outtake != null && robot.outtake.vision != null) {
            double odoGoalDesired = turret.getOdoGoalDesiredHeadingDeg(robot.outtake.vision);
            double odoGoalDelta = turret.getOdoGoalHeadingDeltaDeg(robot.outtake.vision);
            if (Double.isNaN(odoGoalDesired) || Double.isNaN(odoGoalDelta)) {
                telemetry.addData("ODO Goal Desired (deg)", "N/A");
                telemetry.addData("ODO Goal Delta (deg)", "N/A");
            } else {
                telemetry.addData("ODO Goal Desired (deg)", "%.1f", odoGoalDesired);
                telemetry.addData("ODO Goal Delta (deg)", "%.2f", odoGoalDelta);
            }
        }
//        if (robot != null && robot.outtake != null && robot.outtake.vision != null) {
//            int requiredTagId = robot.outtake.vision.getRequiredTagId();
//            boolean seesRequired = robot.outtake.vision.hasRequiredTarget();
//            double tx = requiredTagId >= 0
//                    ? robot.outtake.vision.getTxForTag(requiredTagId)
//                    : robot.outtake.vision.getTx();
//            double distanceIn = requiredTagId >= 0
//                    ? robot.outtake.vision.getDistanceInchesForTag(requiredTagId)
//                    : robot.outtake.vision.getDistanceInches();
//
//            telemetry.addData("TX Lock Required Tag", requiredTagId);
//            telemetry.addData("TX Lock Sees Required", seesRequired);
//            telemetry.addData("TX Lock Tx (deg)", tx);
//            telemetry.addData("TX Lock Distance (in)", distanceIn);
//        }
//        telemetry.addData("TX Lock Toggle", "gp1/gp2 START or right-stick-click");
    }
}
