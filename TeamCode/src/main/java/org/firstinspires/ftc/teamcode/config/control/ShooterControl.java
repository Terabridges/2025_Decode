package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class ShooterControl implements Control {

    //---------------- Software ----------------
    Shooter shooter;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector turretLockToggle = new EdgeDetector( () -> shooter.toggleTurretLock());
    EdgeDetector RPMup = new EdgeDetector( () -> shooter.bumpShooterUp());
    EdgeDetector RPMdown = new EdgeDetector( () -> shooter.bumpShooterDown());

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
        turretLockToggle.update(gp1.right_bumper);
        RPMup.update(gp1.dpad_up);
        RPMdown.update(gp1.dpad_down);

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Turret Lock?", shooter.useTurretLock);
        telemetry.addData("Turret Pow", shooter.turretPower);
        telemetry.addData("Target RPM", shooter.getShooterRPM());
        telemetry.addData("Current RPM", shooter.velToRPM(shooter.getShooterVel()));
        telemetry.addData("actual vision error", shooter.vision.getTx());
    }
}
