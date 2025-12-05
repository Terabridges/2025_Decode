package org.firstinspires.ftc.teamcode.config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.ShooterData;
import org.firstinspires.ftc.teamcode.config.utility.Util;

public class Shooter implements Subsystem {

    //---------------- Hardware ----------------!
    public CRServo turret;
    public AnalogInput turretAnalog;
    public AbsoluteAnalogEncoder turretEnc;
    public DcMotorEx flyLeft;
    public DcMotorEx flyRight;
    public Servo hood;
    public Servo light;
    //public TouchSensor hoodSwitch;

    //---------Objects------
    public Vision vision;
    public Util util;
    public ShooterData shooterData;

    //---------------- Software ----------------!
    private final double TICKS_PER_REV = 28.0; // goBILDA 5202/5203
    private final double SHOOTER_GEAR_RATIO = 1.0;
    double hoodDown = 0.0;
    double hoodUp = 0.9;
    public boolean useData = true;
    double maxRPM = 5700;
    public int requiredTagId = -1;
    public boolean useLight = true;
    public String lightColor = "none";
    private int motifTagId = -1;
    private static final double TURRET_OFFSET_DEG = 45.0; // encoder reading when turret faces field +X
    private static final double TURRET_MIN_DEG = 30.0;
    private static final double TURRET_MAX_DEG = 340.0;

    //---------Targets------
    public double turretTarget = 0.0; // degrees in turret frame
    public double turretTargetDeg = 0.0;
    public double targetRPM = 0.0;
    public double turretManualPow = 0.0;

    //---------useBool------
    public boolean useTurretLock = false;
    public boolean useTurretPID = false;
    public boolean shooterShoot = false;
    public boolean manualTurret = false;

    // Hood smoothing
    private double filteredHood = 0.0;
    private static final double HOOD_MAX_STEP = 0.025;  // max hood position change per update
    private static final double HOOD_DEADBAND = 0.001;

    //---------PID------
    public PIDController turretLockController;
    double p1 = 0.01, i1 = 0.012, d1 = 0.00005;
    double inteTolerance1 = 6.0;
    double deadband1 = 0;
    double maxPow1 = 0.15;
    public double turretPower1, error1;
    double lowThresh = 0.01;
    double minPow = 0.07;

    public PIDController turretController;
    double p2 = 0.006, i2 = 0.005, d2 = 0.0;
    double posTolerance2 = 7;
    double inteTolerance2 = 15;
    double maxPow2 = 1;
    public double turretPower2, error2;

    //--------
    public double turretRight = 0.85;
    public double turretLeft = -0.85;
    public double currentTurretPos;
    double upperLimit = 340;
    double lowerLimit = 30;
    public boolean hasDesiredTarget = false;
    public double hoodOffset = 0;
    public double RPMOffset = 0;

    //---------------- Constructor ----------------!
    public Shooter(HardwareMap map, Vision vision) {
        turret = map.get(CRServo.class, "turret");
        flyLeft = map.get(DcMotorEx.class, "fly_left");
        flyRight = map.get(DcMotorEx.class, "fly_right");
        hood = map.get(Servo.class, "hood");
        turretAnalog = map.get(AnalogInput.class, "turret_analog");
        light = map.get(Servo.class, "light");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0.0, 1.0);
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flyRight.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        hood.setDirection(Servo.Direction.FORWARD);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.vision = vision;

        turretLockController = new PIDController(p1, i1, d1);
        turretLockController.setIntegrationBounds(-inteTolerance1, inteTolerance1);

        turretController = new PIDController(p2, i2, d2);
        turretController.setIntegrationBounds(-inteTolerance2, inteTolerance2);
        turretController.setTolerance(posTolerance2);

        util = new Util();
        shooterData = new ShooterData();
    }

    //---------------- Methods ----------------!

    //---------Turret------
    public void setTurretPower(double power) {
        turret.setPower(power);
    }

    public double limitTurretPower(int startLimit, int endLimit) {
        //1 - (pos-start)/(end-start)
        //240 340
        return (1 - ((turretEnc.getCurrentPosition() - startLimit) / (endLimit - startLimit)));
    }

    public void turretManualRightRE() {
        manualTurret = true;
        turretManualPow = turretRight;
    }

    public void turretManualRightFE() {
        manualTurret = false;
        turretManualPow = 0;
    }

    public void turretManualLeftRE() {
        manualTurret = true;
        turretManualPow = turretLeft;
    }

    public void turretManualLeftFE() {
        manualTurret = false;
        turretManualPow = 0;
    }

    public double getTurretPos() {
        return turretEnc.getCurrentPosition();
    }

    public void toggleTurretLock() {
        useTurretLock = !useTurretLock;
    }

    // Angle helpers
    private double wrapDeg(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    private double normalize360(double deg) {
        return ((deg % 360) + 360) % 360;
    }

    private double encoderToLogical(double encoderDeg) {
        return normalize360(encoderDeg - TURRET_OFFSET_DEG);
    }

    private double logicalToEncoder(double logicalDeg) {
        return normalize360(logicalDeg + TURRET_OFFSET_DEG);
    }

    /**
     * Sets a turret target in degrees (chassis/turret frame). Enables turret PID and disables manual.
     */
    public void setTurretTargetDeg(double targetDeg) {
        double encoderTarget = util.clamp(logicalToEncoder(targetDeg), TURRET_MIN_DEG, TURRET_MAX_DEG);
        turretTargetDeg = normalize360(targetDeg);
        turretTarget = encoderTarget;
        useTurretPID = true;
        manualTurret = false;
    }

    /**
     * Coarse field-based aim: point turret from robot pose toward a goal pose (both in field coords).
     */
    public void aimTurretAtFieldPose(double robotX, double robotY, double robotHeadingRad,
                                     double goalX, double goalY) {
        double desiredHeadingRad = Math.atan2(goalY - robotY, goalX - robotX);
        double turretTargetFieldDeg = Math.toDegrees(desiredHeadingRad - robotHeadingRad);
        turretTargetFieldDeg = ((turretTargetFieldDeg % 360) + 360) % 360; // normalize 0..360 to avoid wrap jumps
        setTurretTargetDeg(turretTargetFieldDeg);
        useTurretLock = false;
        manualTurret = false;
    }

    //Blue is 20, Red is 24

    /**
     * Sets the fiducial ID that the turret lock is allowed to track. Use -1 to accept any tag.
     */
    public void setRequiredTagId(int tagId) {
        requiredTagId = tagId;
    }

    /**
     * Sets the fiducial ID used for motif acquisition. Use -1 to accept any tag.
     */
    public void setMotifTagId(int tagId) {
        motifTagId = tagId;
    }

    public double setTurretLockPID(double targetAngle) {
        turretLockController.setPID(p1, i1, d1);
        error1 = vision.getTx();
        if (Math.abs(error1) < deadband1) error1 = 0.0;
        turretPower1 = turretLockController.calculate(error1, targetAngle);
        turretPower1 = util.clamp(turretPower1, -maxPow1, maxPow1);
        if (turretPower1 > lowThresh){
            if (turretPower1 < minPow){
                turretPower1 = minPow;
            }
        } else if (turretPower1 < -lowThresh){
            if (turretPower1 > -minPow){
                turretPower1 = -minPow;
            }
        }
        return turretPower1;
    }

    public double setTurretPID(double targetAngleDeg) {
        if (targetAngleDeg > upperLimit){targetAngleDeg = upperLimit;}
        if (targetAngleDeg < lowerLimit){targetAngleDeg = lowerLimit;}
        turretController.setPID(p2, i2, d2);
        double currentDeg = turretEnc.getCurrentPosition(); // 0..360 from analog encoder
        double clampedTarget = util.clamp(targetAngleDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
        double errorDeg = clampedTarget - currentDeg; // avoid wrapping through hard limit
        error2 = errorDeg;
        turretPower2 = turretController.calculate(error2, 0.0); // drive error to zero
        turretPower2 = util.clamp(turretPower2, -maxPow2, maxPow2);
        return turretPower2;
    }

    boolean pastPosLimit() {
        return (turretEnc.getCurrentPosition() <= lowerLimit);
    }

    boolean pastNegLimit() {
        return (turretEnc.getCurrentPosition() >= upperLimit);
    }

    //---------Shooter------
    public void setShooterRPM(double RPM) {
        double targetVelocity = RPMToVel(RPM);
        flyLeft.setVelocity(targetVelocity);
        flyRight.setVelocity(targetVelocity);
    }

    public void setShooterVel(double tps) {
        flyLeft.setVelocity(tps);
        flyRight.setVelocity(tps);
    }

    public double getShooterVel() {
        return flyRight.getVelocity();
    }

    public double getShooterRPM() {
        return velToRPM(flyRight.getVelocity());
    }

    public double velToRPM(double tps) {
        double rps = tps / (TICKS_PER_REV * SHOOTER_GEAR_RATIO);
        return rps * 60;
    }

    public double RPMToVel(double rpm) {
        double tpm = rpm * TICKS_PER_REV * SHOOTER_GEAR_RATIO;
        return tpm / 60;
    }

    public void bumpShooterUp() {
        targetRPM += 1000;
        if (targetRPM == 6000) {
            targetRPM = 0;
        }
    }

    public void bumpShooterDown() {
        targetRPM -= 1000;
        if (targetRPM == -1000) {
            targetRPM = 5000;
        }
    }

    public void toggleShooter() {
        shooterShoot = !shooterShoot;
    }

    public boolean isAtRPM() {
        if (targetRPM > 4000) {
            return (Math.abs(targetRPM - getShooterRPM()) < 450);
        } else {
            return (Math.abs(targetRPM - getShooterRPM()) < 300);
        }
    }

    //---------Hood------
    public void setHoodPos(double pos) {
        hood.setPosition(pos);
    }

    public double getHoodPos() {
        return hood.getPosition();
    }

    //---------------
    public double getColorPWN(String color) {
        if (color.equals("red")) {
            return 0.280;
        } else if (color.equals("green")) {
            return 0.470;
        } else if (color.equals("blue")) {
            return 0.611;
        } else {
            return 0;
        }
    }

    public boolean isFarShot(){
        //return (vision.getDistanceInches() > 90);
        return true;
    }

    public String getShooterType(){
        return "TeleOp";
    }

    public void bumpUpHoodOffset(){
        hoodOffset += 0.05;
    }

    public void bumpDownHoodOffset(){
        hoodOffset -= 0.05;
    }

    public void bumpUpRPMOffset(){
        RPMOffset += 25;
    }

    public void bumpDownRPMOffset(){
        RPMOffset -= 25;
    }

    //---------------- Interface Methods ----------------!
    @Override
    public void toInit(){
        if (GlobalVariables.allianceColor.equals("red")) {
            setRequiredTagId(24);
        } else if (GlobalVariables.allianceColor.equals("blue")) {
            setRequiredTagId(20);
        }
    }

    @Override
    public void update() {

        hasDesiredTarget = vision != null && vision.hasTarget() && (requiredTagId < 0 || vision.getCurrentTagId() == requiredTagId);

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
            double lockPower = setTurretLockPID(0.0);
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

            // Lookup directly from table to preserve accuracy, then smooth the hood motion only
            double rpmVal = util.clamp(shooterData.getRPMVal(rawDist), 0, maxRPM);
            double angleVal = util.clamp(shooterData.getAngleVal(rawDist), hoodDown, hoodUp);
            if (rpmVal != -2) {
                targetRPM = rpmVal;
            }
            if (angleVal != -2) {
                // Slew-limit hood to avoid jitter
                if (Math.abs(angleVal - filteredHood) > HOOD_DEADBAND) {
                    double delta = angleVal - filteredHood;
                    double step = Math.max(-HOOD_MAX_STEP, Math.min(HOOD_MAX_STEP, delta));
                    filteredHood += step;
                }
                if (useTurretLock) {
                    setHoodPos(filteredHood+hoodOffset);
                }
            }
        }

        if (shooterShoot) {
            setShooterRPM(targetRPM+RPMOffset);
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
}
