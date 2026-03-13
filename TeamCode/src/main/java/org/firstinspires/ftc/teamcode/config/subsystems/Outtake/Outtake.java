package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.config.utility.ShooterData;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@Configurable
@PsiKitFieldAutoLog
public class Outtake implements Subsystem {
    public enum AimSource {
        NONE,
        ODO
    }

    public enum AimTarget {
        GOAL,
        OBELISK
    }

    //---------------- Hardware ----------------
    public Shooter shooter;
    public Turret turret;
    public Vision vision;
    private ShooterData shooterData;
    public double distanceInches = 0;

    //---------------- Software ----------------
    public static double blueGoalX = 12.0; //0
    public static double blueGoalY = 132.0; //144
    public static double redGoalX = 132.0; //144
    public static double redGoalY = 132.0; //144
    public static double obeliskX = 72.0;
    public static double obeliskY = 144.0;
    public static double turretAimCommandOffsetDeg = 0.0;
    public static double odoAimDirection = -1.0;
    public static boolean enableMovingShotLead = true;
    public static int leadIterations = 10;
    public static double movingLeadSpeedThresholdInS = 3.0;
    public static double launchRobotSizeIn = 17.0;
    public static double bigLaunchApexX = 72.0;
    public static double bigLaunchApexY = 72.0;
    public static double smallLaunchLeftBaseX = 48.0;
    public static double smallLaunchRightBaseX = 96.0;
    public static double smallLaunchApexY = 24.0;
    public static boolean enableRpmRecoilComp = true;
    public static double recoilCompGainPerRPM = 0.00000625;
    public static double recoilCompDeadbandRPM = 30.0;
    public static double recoilCompMaxHoodDelta = 0.08;
    public static double headingOffsetStepDeg = 1.0;
    public static double headingOffsetMaxAbsDeg = 35.0;

    private boolean aimLockEnabled = false;
    private boolean preventTurretWrap = false;
    private AimSource activeAimSource = AimSource.NONE;
    private AimTarget aimTarget = AimTarget.GOAL;
    private double lastRecoilRpmError = 0.0;
    private double lastRecoilHoodDelta = 0.0;
    private double lastBaseHoodPos = 0.0;
    private double lastCompedHoodPos = 0.0;

    public String currentOffsetType = "heading";

    //---------------- Constructor ----------------
    public Outtake(HardwareMap map) {
        shooter = new Shooter(map);
        turret = new Turret(map);
        vision = new Vision(map, turret);
        shooterData = new ShooterData();
    }

    //---------------- Methods ----------------
    public void toggleAimLock() {
        setAimLockEnabled(!isAimLockEnabled());
    }

    public void setAimLockEnabled(boolean enabled) {
        aimLockEnabled = enabled;
        if (!enabled) {
            activeAimSource = AimSource.NONE;
        }
    }

    public boolean isAimLockEnabled() {
        return aimLockEnabled;
    }

    public void setPreventTurretWrap(boolean prevent) {
        preventTurretWrap = prevent;
    }

    public boolean isPreventTurretWrap() {
        return preventTurretWrap;
    }

    public double getLastRecoilRpmError() {
        return lastRecoilRpmError;
    }

    public double getLastRecoilHoodDelta() {
        return lastRecoilHoodDelta;
    }

    public double getLastBaseHoodPos() {
        return lastBaseHoodPos;
    }

    public double getLastCompedHoodPos() {
        return lastCompedHoodPos;
    }

    public static double getTotalTurretAimCommandOffsetDeg() {
        return turretAimCommandOffsetDeg;
    }

    public static void resetTurretAimOffsets() {
        turretAimCommandOffsetDeg = 0.0;
    }

    public AimSource getActiveLockSource() {
        return activeAimSource;
    }

    public AimTarget getAimTarget() {
        return aimTarget;
    }

    public void setAimTargetGoal() {
        aimTarget = AimTarget.GOAL;
    }

    public void setAimTargetObelisk() {
        aimTarget = AimTarget.OBELISK;
    }

    public void updateAimLock() {
        if (!aimLockEnabled) {
            activeAimSource = AimSource.NONE;
            return;
        }

        if (aimTarget == AimTarget.OBELISK) {
            aimAtObeliskWithOdometry();
        } else {
            aimAtGoalWithOdometry();
        }

    }

    public void aimAtObeliskWithOdometry() {
        Pose pose = (follower != null) ? follower.getPose() : null;
        if (pose == null) {
            activeAimSource = AimSource.NONE;
            return;
        }
        activeAimSource = AimSource.ODO;
        aimAtFieldPoint(pose, obeliskX, obeliskY);
    }

    public void aimAtGoalWithOdometry() {
        Pose pose = (follower != null) ? follower.getPose() : null;
        if (pose == null) {
            activeAimSource = AimSource.NONE;
            return;
        }
        activeAimSource = AimSource.ODO;

        if (GlobalVariables.isBlueAlliance()) {
            aimAtFieldPoint(pose, blueGoalX, blueGoalY);
        } else {
            aimAtFieldPoint(pose, redGoalX, redGoalY);
        }
    }

    public boolean isVisionOnTarget(Vision vision, double toleranceDeg) {
        if (vision == null) {
            return false;
        }
        if (!vision.hasRequiredTarget()) {
            return false;
        }

        double txDeg = vision.getTxForTag(vision.getRequiredTagId());
        if (!Double.isFinite(txDeg)) {
            return false;
        }

        return Math.abs(txDeg) <= Math.abs(toleranceDeg);
    }

    /**
     * Computes the turret command needed to hit the alliance goal from a supplied field pose.
     */
    public double computeGoalTurretDegFromPose(Pose robotPose) {
        if (robotPose == null) {
            return Double.NaN;
        }
        double targetX = GlobalVariables.isBlueAlliance() ? blueGoalX : redGoalX;
        double targetY = GlobalVariables.isBlueAlliance() ? blueGoalY : redGoalY;
        double desiredDeg = computeFieldPointTurretDeg(robotPose, targetX, targetY);
        return turret.normalizeDegrees(desiredDeg + getTotalTurretAimCommandOffsetDeg());
    }

    /**
     * Commands turret to the alliance-goal angle computed from the supplied field pose.
     */
    public void commandGoalTurretFromPose(Pose robotPose) {
        double desiredDeg = computeGoalTurretDegFromPose(robotPose);
        if (Double.isFinite(desiredDeg)) {
            commandTurretDegree(desiredDeg);
        }
    }

    private void aimAtFieldPoint(Pose robotPose, double targetX, double targetY) {
        double desiredDeg = computeFieldPointTurretDeg(robotPose, targetX, targetY);
        commandTurretDegree(desiredDeg + getTotalTurretAimCommandOffsetDeg());
    }

    private void commandTurretDegree(double desiredDeg) {
        if (preventTurretWrap) {
            turret.setTurretDegreeNoWrap(desiredDeg);
        } else {
            turret.setTurretDegree(desiredDeg);
        }
    }

    private double computeFieldPointTurretDeg(Pose robotPose, double targetX, double targetY) {
        double[] leadVector = computeLeadAdjustedVector(robotPose, targetX, targetY);
        double dx = leadVector[0];
        double dy = leadVector[1];

        double headingToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        double relativeDeg = wrapSignedDegrees(headingToTargetDeg - robotHeadingDeg);
        return turret.normalizeDegrees(Turret.turretForwardDeg + (odoAimDirection * relativeDeg));
    }

    private double computeLeadAdjustedDistance(Pose robotPose, double targetX, double targetY) {
        return computeLeadAdjustedVector(robotPose, targetX, targetY)[2];
    }

    private double[] computeLeadAdjustedVector(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        double distance = Math.hypot(dx, dy);

        if (enableMovingShotLead && follower != null && follower.getVelocity() != null) {
            double vX = follower.getVelocity().getXComponent();
            double vY = follower.getVelocity().getYComponent();
            int iterations = Math.max(1, leadIterations);

            for (int i = 0; i < iterations; i++) {
                double shotTime = shooterData.getShotTimeVal(distance);
                dx = targetX - robotPose.getX() - (vX * shotTime);
                dy = targetY - robotPose.getY() - (vY * shotTime);
                distance = Math.hypot(dx, dy);
            }
        }

        return new double[]{dx, dy, distance};
    }

    /**
     * Returns true when any portion of the 17x17in robot footprint intersects either launch zone.
     * Pose is treated as robot center (Pedro/Pinpoint convention).
     */
    public boolean isAnyPartInLaunchZone() {
        Pose pose = (follower != null) ? follower.getPose() : null;
        if (pose == null) {
            return false;
        }
        return isAnyPartInLaunchZone(pose);
    }

    public boolean isAnyPartInLaunchZone(Pose robotPose) {
        if (robotPose == null) {
            return false;
        }
        double[][] robotFootprint = buildRobotSquare(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), launchRobotSizeIn);
        return polygonIntersects(robotFootprint, getBigLaunchTriangle())
                || polygonIntersects(robotFootprint, getSmallLaunchTriangle());
    }

    private boolean isRobotMovingForLead() {
        if (follower == null || follower.getVelocity() == null) {
            return false;
        }
        return follower.getVelocity().getMagnitude() > movingLeadSpeedThresholdInS;
    }

    private double[] getActiveAimTargetFieldPoint() {
        if (aimTarget == AimTarget.OBELISK) {
            return new double[]{obeliskX, obeliskY};
        }
        if (GlobalVariables.isBlueAlliance()) {
            return new double[]{blueGoalX, blueGoalY};
        }
        return new double[]{redGoalX, redGoalY};
    }

    private double wrapSignedDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private double[][] getBigLaunchTriangle() {
        return new double[][]{
                {0.0, 144.0},
                {bigLaunchApexX, bigLaunchApexY},
                {144.0, 144.0}
        };
    }

    private double[][] getSmallLaunchTriangle() {
        return new double[][]{
                {smallLaunchLeftBaseX, 0.0},
                {bigLaunchApexX, smallLaunchApexY},
                {smallLaunchRightBaseX, 0.0}
        };
    }

    private double[][] buildRobotSquare(double cx, double cy, double headingRad, double sizeIn) {
        double half = Math.abs(sizeIn) * 0.5;
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double[][] local = new double[][]{
                {-half, -half},
                {half, -half},
                {half, half},
                {-half, half}
        };

        double[][] world = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double lx = local[i][0];
            double ly = local[i][1];
            world[i][0] = cx + (lx * cos - ly * sin);
            world[i][1] = cy + (lx * sin + ly * cos);
        }
        return world;
    }

    private boolean polygonIntersects(double[][] polyA, double[][] polyB) {
        for (double[] p : polyA) {
            if (pointInConvexPolygon(polyB, p[0], p[1])) {
                return true;
            }
        }
        for (double[] p : polyB) {
            if (pointInConvexPolygon(polyA, p[0], p[1])) {
                return true;
            }
        }

        for (int i = 0; i < polyA.length; i++) {
            double[] a1 = polyA[i];
            double[] a2 = polyA[(i + 1) % polyA.length];
            for (int j = 0; j < polyB.length; j++) {
                double[] b1 = polyB[j];
                double[] b2 = polyB[(j + 1) % polyB.length];
                if (segmentsIntersect(a1[0], a1[1], a2[0], a2[1], b1[0], b1[1], b2[0], b2[1])) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean pointInConvexPolygon(double[][] poly, double x, double y) {
        if (poly == null || poly.length < 3) {
            return false;
        }
        double sign = 0.0;
        for (int i = 0; i < poly.length; i++) {
            double[] a = poly[i];
            double[] b = poly[(i + 1) % poly.length];
            double cross = cross2d(b[0] - a[0], b[1] - a[1], x - a[0], y - a[1]);
            if (Math.abs(cross) < 1e-9) {
                continue;
            }
            if (sign == 0.0) {
                sign = Math.signum(cross);
            } else if (Math.signum(cross) != sign) {
                return false;
            }
        }
        return true;
    }

    private boolean segmentsIntersect(double ax, double ay, double bx, double by,
                                      double cx, double cy, double dx, double dy) {
        double o1 = orient(ax, ay, bx, by, cx, cy);
        double o2 = orient(ax, ay, bx, by, dx, dy);
        double o3 = orient(cx, cy, dx, dy, ax, ay);
        double o4 = orient(cx, cy, dx, dy, bx, by);

        if ((o1 * o2 < 0.0) && (o3 * o4 < 0.0)) {
            return true;
        }

        double eps = 1e-9;
        if (Math.abs(o1) < eps && onSegment(ax, ay, bx, by, cx, cy)) return true;
        if (Math.abs(o2) < eps && onSegment(ax, ay, bx, by, dx, dy)) return true;
        if (Math.abs(o3) < eps && onSegment(cx, cy, dx, dy, ax, ay)) return true;
        if (Math.abs(o4) < eps && onSegment(cx, cy, dx, dy, bx, by)) return true;
        return false;
    }

    private double orient(double ax, double ay, double bx, double by, double cx, double cy) {
        return cross2d(bx - ax, by - ay, cx - ax, cy - ay);
    }

    private double cross2d(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    private boolean onSegment(double ax, double ay, double bx, double by, double px, double py) {
        return px >= Math.min(ax, bx) - 1e-9
                && px <= Math.max(ax, bx) + 1e-9
                && py >= Math.min(ay, by) - 1e-9
                && py <= Math.max(ay, by) + 1e-9;
    }
    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        resetTurretAimOffsets();
        shooter.toInit();
        turret.toInit();
        vision.toInit();
    }

    @Override
    public void update(){
        vision.update();

        Pose pose = (follower != null) ? follower.getPose() : null;
        if (pose != null) {
            double[] targetPoint = getActiveAimTargetFieldPoint();
            distanceInches = computeLeadAdjustedDistance(pose, targetPoint[0], targetPoint[1]);
        }

        shooter.flywheelTargetRPM = shooterData.getRPMVal(distanceInches);
        double baseHoodPos = shooterData.getAngleVal(distanceInches);
        lastBaseHoodPos = baseHoodPos;
        shooter.hoodPos = applyRpmRecoilComp(baseHoodPos);
        lastCompedHoodPos = shooter.hoodPos;
        shooter.update();
        turret.update();
        updateAimLock();
    }

    private double applyRpmRecoilComp(double baseHoodPos) {
        lastRecoilRpmError = shooter.getTargetRPM() - shooter.getCurrentRPM();
        lastRecoilHoodDelta = 0.0;

        if (!enableRpmRecoilComp || !shooter.useFlywheelPID || !shooter.autoHood) {
            return clamp01(baseHoodPos);
        }

        if (Math.abs(lastRecoilRpmError) <= recoilCompDeadbandRPM) {
            return clamp01(baseHoodPos);
        }

        // Positive RPM error means flywheel is under target, so raise hood angle.
        double hoodDelta = recoilCompGainPerRPM * lastRecoilRpmError;
        hoodDelta = Math.max(-recoilCompMaxHoodDelta, Math.min(recoilCompMaxHoodDelta, hoodDelta));
        lastRecoilHoodDelta = hoodDelta;
        return clamp01(baseHoodPos + hoodDelta);
    }

    private double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    public void increaseOffset(){
        if (currentOffsetType.equals("heading")) {
            turretAimCommandOffsetDeg += headingOffsetStepDeg;
            turretAimCommandOffsetDeg = Math.max(-headingOffsetMaxAbsDeg, Math.min(headingOffsetMaxAbsDeg, turretAimCommandOffsetDeg));
        } else if (currentOffsetType.equals("rpm")) {
            shooter.flywheelOffset += 25;
        } else if (currentOffsetType.equals("hood")){
            shooter.hoodOffset += 0.05;
        }
    }

    public void decreaseOffset(){
        if (currentOffsetType.equals("heading")) {
            turretAimCommandOffsetDeg -= headingOffsetStepDeg;
            turretAimCommandOffsetDeg = Math.max(-headingOffsetMaxAbsDeg, Math.min(headingOffsetMaxAbsDeg, turretAimCommandOffsetDeg));
        } else if (currentOffsetType.equals("rpm")) {
            shooter.flywheelOffset -= 25;
        } else if (currentOffsetType.equals("hood")){
            shooter.hoodOffset -= 0.05;
        }
    }

    public void changeOffsetType(){
        if (currentOffsetType.equals("heading")) {
            currentOffsetType = "rpm";
        } else if (currentOffsetType.equals("rpm")) {
            currentOffsetType = "hood";
        } else if (currentOffsetType.equals("hood")){
            currentOffsetType = "heading";
        }
    }

    public void resetOffset(){
        if (currentOffsetType.equals("heading")) {
            turretAimCommandOffsetDeg = 0;
        } else if (currentOffsetType.equals("rpm")) {
            shooter.flywheelOffset = 0;
        } else if (currentOffsetType.equals("hood")){
            shooter.hoodOffset = 0;
        }
    }
}
