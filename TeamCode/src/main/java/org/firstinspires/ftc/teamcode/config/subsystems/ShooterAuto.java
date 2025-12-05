package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

/**
 * Auto-mode shooter variant that layers the new behavior on top of the base Shooter API.
 */
public class ShooterAuto extends Shooter {

    private double prevFlywheelVel = 0.0;
    private long prevFlywheelTimeNs = System.nanoTime();
    private final double kFlywheelAccelFF = 1e-6; // scales flywheel accel (ticks/s^2) into turret power
    private final double flywheelAccelThreshold = 1000.0;
    private double filteredTx = 0.0;

    // Local hood smoothing because base filteredHood is private
    private double filteredHoodAuto = 0.0;
    private static final double HOOD_MAX_STEP_AUTO = 0.025;
    private static final double HOOD_DEADBAND_AUTO = 0.001;
    private double maxPow = 0.3;
    protected double lastValidDistanceInches = 0.0;

    public ShooterAuto(HardwareMap map, Vision vision) {
        super(map, vision);
        // Retune lock PID for the auto behavior
        p1 = 0.035;
        i1 = 0.03;
        d1 = 0.0;
        inteTolerance1 = 8;
        deadband1 = 0.3;
        turretLockController.setPID(p1, i1, d1);
        turretLockController.setIntegrationBounds(-inteTolerance1, inteTolerance1);
    }

    @Override
    public double setTurretLockPID(double targetAngle) {
        turretLockController.setPID(p1, i1, d1);
        error1 = targetAngle;
        if (Math.abs(error1) < deadband1) error1 = 0.0;
        turretPower1 = turretLockController.calculate(error1, targetAngle);
        turretPower1 = util.clamp(turretPower1, -maxPow, maxPow);
        if (turretPower1 > lowThresh) {
            if (turretPower1 < minPow) {
                turretPower1 = minPow;
            }
        } else if (turretPower1 < -lowThresh) {
            if (turretPower1 > -minPow) {
                turretPower1 = -minPow;
            }
        }
        return turretPower1;
    }

    @Override
    public boolean isFarShot() {
        return true;
    }

    @Override
    public String getShooterType(){
        return "Auto";
    }

    @Override
    public void update() {

        hasDesiredTarget = vision != null && vision.hasTarget() && (requiredTagId < 0 || vision.getCurrentTagId() == requiredTagId);
        double flywheelFF = getFlywheelAccelFF();
        double filteredTxVal = getFilteredTx();

        // Turret control priority: manual -> lock -> PID -> idle
        if (manualTurret) {
            if (turretManualPow > 0) {
                if (getTurretPos() <= 130) {
                    setTurretPower(turretManualPow * limitTurretPower(130, 30));
                } else {
                    setTurretPower(turretManualPow);
                }
            } else if (turretManualPow < 0) {
                if (getTurretPos() >= 240) {
                    setTurretPower(turretManualPow * limitTurretPower(240, 340));
                } else {
                    setTurretPower(turretManualPow);
                }
            } else {
                setTurretPower(0);
            }
        } else if (useTurretLock && hasDesiredTarget) {
            double lockPower = setTurretLockPID(filteredTxVal) + flywheelFF;
            if (Math.abs(filteredTxVal) < 3.0) {
                lockPower *= 0.5;
            }
            double pos = getTurretPos();
            if (lockPower > 0) {
                if (pos <= lowerLimit) {
                    lockPower = 0.0;
                } else if (pos <= 130) {
                    lockPower *= limitTurretPower(130, 30);
                }
            } else if (lockPower < 0) {
                if (pos >= upperLimit) {
                    lockPower = 0.0;
                } else if (pos >= 240) {
                    lockPower *= limitTurretPower(240, 340);
                }
            }
            setTurretPower(lockPower);
        } else if (useTurretPID) {
            setTurretPower(setTurretPID(turretTarget));
        } else {
            setTurretPower(0);
        }

        if (useData && hasDesiredTarget) {
            double rawDist = vision.getDistanceInches();
            boolean haveDistance = rawDist > 0.1;
            if (haveDistance) {
                lastValidDistanceInches = rawDist;
            } else if (lastValidDistanceInches > 0.0) {
                rawDist = lastValidDistanceInches; // fallback to last good reading to avoid hood dip
                haveDistance = true;
            }

            // Lookup directly from table to preserve accuracy, then smooth the hood motion only
            double rpmVal = haveDistance ? util.clamp(shooterData.getRPMVal(rawDist), 0, maxRPM) : -2;
            double angleVal = haveDistance ? util.clamp(shooterData.getAngleVal(rawDist), hoodDown, hoodUp) : -2;
            if (rpmVal != -2) {
                targetRPM = rpmVal;
            }
            if (angleVal != -2) {
                // Slew-limit hood to avoid jitter
                if (Math.abs(angleVal - filteredHoodAuto) > HOOD_DEADBAND_AUTO) {
                    double delta = angleVal - filteredHoodAuto;
                    double step = Math.max(-HOOD_MAX_STEP_AUTO, Math.min(HOOD_MAX_STEP_AUTO, delta));
                    filteredHoodAuto += step;
                }
                if (useTurretLock) {
                    setHoodPos(filteredHoodAuto);
                }
            }
        }

        if (shooterShoot) {
            setShooterRPM(targetRPM);
        } else {
            setShooterRPM(0);
        }

        if (vision.getTx() != 0 && Math.abs(vision.getTx()) < 3 && hasDesiredTarget && useTurretLock){
            lightColor = "green";
        } else {
            if(GlobalVariables.allianceColor.equals("red")) {
                lightColor = "red";
            } else if (GlobalVariables.allianceColor.equals("blue")){
                lightColor = "blue";
            }
        }

        light.setPosition(getColorPWN(lightColor));
    }

    private double getFilteredTx() {
        double raw = (vision != null) ? vision.getTx() : 0.0;
        // simple low-pass filter to reduce jitter
        filteredTx = 0.7 * filteredTx + 0.3 * raw;
        return filteredTx;
    }

    private double getFlywheelAccelFF() {
        long now = System.nanoTime();
        double dt = (now - prevFlywheelTimeNs) / 1e9;
        double vel = getShooterVel();
        double accel = 0.0;
        if (dt > 1e-4) {
            accel = (vel - prevFlywheelVel) / dt;
        }
        prevFlywheelVel = vel;
        prevFlywheelTimeNs = now;
        if (!shooterShoot || Math.abs(accel) < flywheelAccelThreshold) {
            return 0.0;
        }
        return kFlywheelAccelFF * accel;
    }
}
