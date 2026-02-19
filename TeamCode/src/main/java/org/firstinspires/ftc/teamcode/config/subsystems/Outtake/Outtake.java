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
    private ShooterData shooterData;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Outtake(HardwareMap map) {
        shooter = new Shooter(map);
        turret = new Turret(map);
        vision = new Vision(map);
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
        shooter.flywheelTargetRPM = shooterData.getRPMVal(vision.getDistanceInches());
        shooter.hoodPos = shooterData.getAngleVal(vision.getDistanceInches());
        shooter.update();
        turret.update();
        vision.update();
        turret.updateTxLock(vision);
    }

}
