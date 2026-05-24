package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.config.utility.Util;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@Configurable
@PsiKitFieldAutoLog
public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private final Servo leftTurret;
    private final Servo rightTurret;
    private final AnalogInput turretAnalog;
    private final AbsoluteAnalogEncoder turretEnc;
    private final Util util;

    //---------------- Software ----------------
    public static double turretMinDeg = 16.418;
    public static double turretMaxDeg = 328.418;
    public static double turretForwardDeg = 198.055;
    public static double turretVelocity = 0.0;
    public static double velocityLoopTime = 250.0;

    public static boolean invertRightServo = false;
    public static double rightServoOffset = 0.021;

    public static double turretServoRefPos = 0.50;
    public static double turretServoRefTurretDeg = 180.0;
    public static double turretDegPerServoCommand = -341.4;
    public static double turretServoPwmMinUs = 500.0;
    public static double turretServoPwmMaxUs = 2500.0;

    public static double encoderRefTurretDeg = turretForwardDeg;
    public static double encoderRefDeg = 200.0;
    public static double encoderToTurretScale = 1.0;
    public static boolean encoderDirectionInverted = false;
    public static double turretWrapCooldownSec = 0.5;

    private double commandedTurretDeg = 180.0;
    private long lastTurretWrapNs = Long.MIN_VALUE;

    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        leftTurret = map.get(Servo.class, "turretL");
        rightTurret = map.get(Servo.class, "turretR");
        applyTurretServoPwmRange(leftTurret);
        applyTurretServoPwmRange(rightTurret);

        turretAnalog = map.get(AnalogInput.class, "turretAnalog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        util = new Util();

        syncCommandToMeasured();
    }

    //---------------- Core movement ----------------
    public void setTurretPos(double pos) {
        double basePos = clampBasePosToSharedRange(pos);
        commandedTurretDeg = baseServoPosToTurretDeg(basePos);

        double leftPos = util.clamp(basePos, 0.0, 1.0);
        double rightBasePos = invertRightServo ? (1.0 - basePos) : basePos;
        double rightPos = util.clamp(rightBasePos + rightServoOffset, 0.0, 1.0);

        leftTurret.setPosition(leftPos);
        rightTurret.setPosition(rightPos);
    }

    public void setTurretDegree(double degree) {
        double normalized = normalizeDegrees(degree);
        double clamped = clampToSafeRange(normalized);
        double current = normalizeDegrees(getCurrentDegrees());
        if (Math.abs(clamped - current) > 180.0) {
            if (!isTurretWrapCooldownReady()) {
                holdNearestTurretLimit(current);
                return;
            }
            lastTurretWrapNs = System.nanoTime();
        }
        setTurretPos(turretDegToBaseServoPos(clamped));
    }

    /**
     * Commands turret angle, but if getting there would require a 0/360 wrap move,
     * hold at the nearest physical limit instead.
     */
    public void setTurretDegreeNoWrap(double degree) {
        double normalized = normalizeDegrees(degree);
        double clampedTarget = clampToSafeRange(normalized);
        double current = normalizeDegrees(getCurrentDegrees());

        if (Math.abs(clampedTarget - current) > 180.0) {
            holdNearestTurretLimit(current);
            return;
        }

        setTurretPos(turretDegToBaseServoPos(clampedTarget));
    }

    private boolean isTurretWrapCooldownReady() {
        if (lastTurretWrapNs == Long.MIN_VALUE) {
            return true;
        }
        double elapsedSec = (System.nanoTime() - lastTurretWrapNs) / 1_000_000_000.0;
        return elapsedSec >= Math.max(0.0, turretWrapCooldownSec);
    }

    private void holdNearestTurretLimit(double current) {
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        double holdLimit = (Math.abs(current - maxDeg) <= Math.abs(current - minDeg)) ? maxDeg : minDeg;
        setTurretPos(turretDegToBaseServoPos(holdLimit));
    }

    public double getCurrentDegrees() {
        return commandedTurretDeg;
    }

    public double normalizeDegrees(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    public double clampToSafeRange(double degrees) {
        double minDeg = Math.min(turretMinDeg, turretMaxDeg);
        double maxDeg = Math.max(turretMinDeg, turretMaxDeg);
        return util.clamp(degrees, minDeg, maxDeg);
    }

    public boolean atMinLimit(double marginDeg) {
        return getCurrentDegrees() <= (Math.min(turretMinDeg, turretMaxDeg) + marginDeg);
    }

    public boolean atMaxLimit(double marginDeg) {
        return getCurrentDegrees() >= (Math.max(turretMinDeg, turretMaxDeg) - marginDeg);
    }

    private double turretDegToBaseServoPos(double turretDeg) {
        double slope = (Math.abs(turretDegPerServoCommand) < 1e-6) ? 360.0 : turretDegPerServoCommand;
        double errorDeg = wrapSignedDegrees(normalizeDegrees(turretDeg) - normalizeDegrees(turretServoRefTurretDeg));
        return clampBasePosToSharedRange(turretServoRefPos + (errorDeg / slope));
    }

    private double baseServoPosToTurretDeg(double basePos) {
        double slope = (Math.abs(turretDegPerServoCommand) < 1e-6) ? 360.0 : turretDegPerServoCommand;
        return normalizeDegrees(turretServoRefTurretDeg + ((basePos - turretServoRefPos) * slope));
    }

    private double getSharedBaseMin() {
        return Math.max(0.0, invertRightServo ? rightServoOffset : -rightServoOffset);
    }

    private double getSharedBaseMax() {
        return Math.min(1.0, invertRightServo ? 1.0 + rightServoOffset : 1.0 - rightServoOffset);
    }

    private double clampBasePosToSharedRange(double requestedBasePos) {
        return util.clamp(requestedBasePos, getSharedBaseMin(), getSharedBaseMax());
    }

    private double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private void applyTurretServoPwmRange(Servo servo) {
        if (!(servo instanceof PwmControl)) {
            return;
        }
        double lo = Math.min(turretServoPwmMinUs, turretServoPwmMaxUs);
        double hi = Math.max(turretServoPwmMinUs, turretServoPwmMaxUs);
        ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(lo, hi));
    }

    //---------------- Encoder helpers ----------------
    public double getEncoderDegrees() {
        return turretEnc.getCurrentPosition();
    }

    public void syncCommandToMeasured() {
        double mappedEncoderDeg = getMappedEncoderTurretDegrees();
        if (!Double.isNaN(mappedEncoderDeg) && !Double.isInfinite(mappedEncoderDeg)) {
            commandedTurretDeg = normalizeDegrees(mappedEncoderDeg);
            return;
        }

        double leftPos = util.clamp(leftTurret.getPosition(), 0.0, 1.0);
        commandedTurretDeg = baseServoPosToTurretDeg(leftPos);
    }

    public double getMappedEncoderTurretDegrees() {
        double encoderDeg = getEncoderDegrees();
        if (Double.isNaN(encoderDeg) || Double.isInfinite(encoderDeg)) {
            return Double.NaN;
        }
        double deltaEncDeg = wrapSignedDegrees(encoderDeg - encoderRefDeg);
        if (encoderDirectionInverted) {
            deltaEncDeg = -deltaEncDeg;
        }
        double scale = (Math.abs(encoderToTurretScale) < 1e-6) ? 1.0 : encoderToTurretScale;
        return normalizeDegrees(encoderRefTurretDeg + (deltaEncDeg * scale));
    }

    public double getMappedEncoderErrorDeg(double targetTurretDeg) {
        double mapped = getMappedEncoderTurretDegrees();
        if (Double.isNaN(mapped)) {
            return Double.NaN;
        }
        return wrapSignedDegrees(mapped - normalizeDegrees(targetTurretDeg));
    }

    public double getEncoderVoltage() {
        return turretAnalog.getVoltage();
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit() {
        syncCommandToMeasured();
    }

    @Override
    public void update() {
        // Movement is explicit via setTurretDegree/setTurretPos.
    }

    @Override
    public void logPsiKitData() {
        double baseServoPosition = turretDegToBaseServoPos(commandedTurretDeg);
        double rightServoPosition = util.clamp((invertRightServo ? (1.0 - baseServoPosition) : baseServoPosition) + rightServoOffset, 0.0, 1.0);

        Logger.recordOutput("Subsystems/Outtake/Turret/CommandedDegree", commandedTurretDeg);
        Logger.recordOutput("Subsystems/Outtake/Turret/ServoBasePosition", baseServoPosition);
        Logger.recordOutput("Subsystems/Outtake/Turret/LeftServoPosition", baseServoPosition);
        Logger.recordOutput("Subsystems/Outtake/Turret/RightServoPosition", rightServoPosition);
        Logger.recordOutput("Subsystems/Outtake/Turret/EncoderDegree", getEncoderDegrees());
        Logger.recordOutput("Subsystems/Outtake/Turret/MappedEncoderDegree", getMappedEncoderTurretDegrees());
        Logger.recordOutput("Subsystems/Outtake/Turret/MappedEncoderErrorDegree", getMappedEncoderErrorDeg(commandedTurretDeg));
        Logger.recordOutput("Subsystems/Outtake/Turret/EncoderVoltage", turretAnalog.getVoltage());
        Logger.recordOutput("Subsystems/Outtake/Turret/AtMinLimit", atMinLimit(0.0));
        Logger.recordOutput("Subsystems/Outtake/Turret/AtMaxLimit", atMaxLimit(0.0));
    }
}
