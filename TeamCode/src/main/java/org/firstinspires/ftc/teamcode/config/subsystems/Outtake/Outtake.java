package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.ShooterData;

public class Outtake implements Subsystem {

    //---------------- Hardware ----------------
    public Shooter shooter;
    public Turret turret;
    public Vision vision;
    public Relocalization relocalization;
    private ShooterData shooterData;
    public double distanceInches = 0;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Outtake(HardwareMap map) {
        shooter = new Shooter(map);
        turret = new Turret(map);
        vision = new Vision(map);
        relocalization = new Relocalization(turret, vision);
        shooterData = new ShooterData();
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        shooter.toInit();
        turret.toInit();
        vision.toInit();
    }

    @Override
    public void update(){
        distanceInches = vision.getDistanceInches();
        shooter.flywheelTargetRPM = shooterData.getRPMVal(distanceInches);
        shooter.hoodPos = shooterData.getAngleVal(distanceInches);
        shooter.update();
        turret.update();
        vision.update();
        relocalization.update();
        turret.updateAimLock(vision);
    }

}
