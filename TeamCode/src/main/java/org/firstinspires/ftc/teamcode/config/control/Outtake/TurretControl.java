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
        toggleAimLockStart.update(gp1.a);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Aim Lock", turret.isAimLockEnabled());
        telemetry.addData("Aim Source", turret.getActiveLockSource());
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
