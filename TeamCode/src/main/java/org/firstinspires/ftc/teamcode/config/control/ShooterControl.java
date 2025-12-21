package org.firstinspires.ftc.teamcode.config.control;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

public class ShooterControl implements Control {

    //---------------- Software ----------------
    Shooter shooter;
    GamepadView gp1;
    GamepadView gp2;
    EdgeDetector turretLockToggle = new EdgeDetector( () -> shooter.toggleTurretLock());
    EdgeDetector toggleShooter = new EdgeDetector( () -> shooter.toggleShooter());
    EdgeDetector bumpUpHoodOffset = new EdgeDetector(()-> shooter.bumpUpHoodOffset());
    EdgeDetector bumpDownHoodOffset = new EdgeDetector(()-> shooter.bumpDownHoodOffset());
    EdgeDetector bumpUpRPMOffset = new EdgeDetector(()-> shooter.bumpUpRPMOffset());
    EdgeDetector bumpDownRPMOffset = new EdgeDetector(()-> shooter.bumpDownRPMOffset());


    //---------------- Constructor ----------------
    public ShooterControl(Shooter shooter, GamepadView gp1, GamepadView gp2){
        this.shooter = shooter;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        turretLockToggle.update(gp1.dpadUp());
        toggleShooter.update(gp1.y());
        if (gp1.rightTrigger() > 0.05){
            shooter.manualTurret = true;
            shooter.turretManualPow = gp1.rightTrigger()/2;
        } else if (gp1.leftTrigger() > 0.05){
            shooter.manualTurret = true;
            shooter.turretManualPow = -gp1.leftTrigger() /2;
        } else if (gp2.rightTrigger() > 0.05){
            shooter.manualTurret = true;
            shooter.turretManualPow = gp2.rightTrigger()/2;
        } else if (gp2.leftTrigger() > 0.05){
            shooter.manualTurret = true;
            shooter.turretManualPow = -gp2.leftTrigger() /2;
        } else {
            shooter.manualTurret = false;
            shooter.turretManualPow = 0;
        }

        bumpUpHoodOffset.update(gp2.y());
        bumpDownHoodOffset.update(gp2.a());
        bumpUpRPMOffset.update(gp2.dpadUp());
        bumpDownRPMOffset.update(gp2.dpadDown());

    }

    @Override
    public void addTelemetry(TelemetrySink telemetry){
        telemetry.addData("Turret Lock?", shooter.useTurretLock);
        telemetry.addData("RPM Tgt / Cur", "%.1f / %.1f", shooter.targetRPM, shooter.getShooterRPM());
        //telemetry.addData("Current RPM", shooter.getShooterRPM());
        telemetry.addData("Motif", GlobalVariables.motif);
        telemetry.addData("Hood Offset", shooter.hoodOffset);
        telemetry.addData("RPM Offset", shooter.RPMOffset);

//        telemetry.addData("Shooter Type", shooter.getShooterType());
//        telemetry.addData("Current Angle", shooter.getHoodPos());
//        telemetry.addData("Current Turret Pos", shooter.getTurretPos());
//        telemetry.addData("Is Far Shot", shooter.isFarShot());
    }

}
