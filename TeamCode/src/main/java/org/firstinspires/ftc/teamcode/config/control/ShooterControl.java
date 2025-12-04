package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

public class ShooterControl implements Control {

    //---------------- Software ----------------
    Shooter shooter;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector turretLockToggle = new EdgeDetector( () -> shooter.toggleTurretLock());
    EdgeDetector toggleShooter = new EdgeDetector( () -> shooter.toggleShooter());


    //---------------- Constructor ----------------
    public ShooterControl(Shooter shooter, Gamepad gp1, Gamepad gp2){
        this.shooter = shooter;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public ShooterControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.shooter, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        turretLockToggle.update(gp1.dpad_up);
        toggleShooter.update(gp1.y);
        if (gp1.right_trigger > 0.05){
            shooter.manualTurret = true;
            shooter.turretManualPow = gp1.right_trigger/2;
        } else if (gp1.left_trigger > 0.05){
            shooter.manualTurret = true;
            shooter.turretManualPow = -gp1.left_trigger /2;
        } else {
            shooter.manualTurret = false;
            shooter.turretManualPow = 0;
        }

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Turret Lock?", shooter.useTurretLock);
        telemetry.addData("Target RPM", shooter.targetRPM);
        telemetry.addData("Current RPM", shooter.getShooterRPM());
        telemetry.addData("Motif", GlobalVariables.motif);
        telemetry.addData("Shooter Type", shooter.getShooterType());

//        telemetry.addData("Current Angle", shooter.getHoodPos());
//        telemetry.addData("Current Turret Pos", shooter.getTurretPos());
//        telemetry.addData("Is Far Shot", shooter.isFarShot());
    }

}
