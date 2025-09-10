package org.firstinspires.ftc.teamcode.config.utility;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * PoseFusion_Subsystem
 * - Maintains a fused pose using Odometry + Limelight (MT1 or MT2).
 * - Early-session calibration estimates a constant Vision->Odom bias (dx,dy,dθ).
 * - Vision is corrected by that bias, then blended with odometry (complementary filter).
 *
 * Units:
 *   - Positions in METERS (field frame).
 *   - Headings in RADIANS, wrapped to (-pi, pi].
 */

//Note: Code from CHATGPT. Untested.

public class PoseFusionWithBias_util {

    // ===== Tunables =====
    private double alphaVision = 0.20;           // vision blend weight when fresh (0..1)
    private long   maxVisionStalenessMs = 120;   // reject stale vision frames
    private double maxAcceptableJumpM   = 0.70;  // reject vision if far from odom
    private double maxAcceptableYawRad  = Math.toRadians(25);
    private long   halfLifeMs           = 250;   // older frames get lower alpha

    // Calibration gates
    private double calMaxTransOutlierM  = 0.50;
    private double calMaxYawOutlierRad  = Math.toRadians(20);

    // ===== Fused state =====
    private double xMeters = 0, yMeters = 0, headingRad = 0;
    private boolean initialized = false;

    // ===== Bias estimator (Vision -> Odom correction) =====
    private final BiasEstimator bias = new BiasEstimator();

    // ===== Public config setters =====
    public void setAlphaVision(double a) { alphaVision = clamp(a, 0, 1); }
    public void setMaxVisionStalenessMs(long ms) { maxVisionStalenessMs = Math.max(0, ms); }
    public void setMaxAcceptableJumpM(double m) { maxAcceptableJumpM = Math.max(0, m); }
    public void setMaxAcceptableYawRad(double r) { maxAcceptableYawRad = Math.max(0, r); }
    public void setHalfLifeMs(long ms) { halfLifeMs = Math.max(1, ms); }

    public void setCalOutlierGates(double maxM, double maxRad) {
        calMaxTransOutlierM = Math.max(0, maxM);
        calMaxYawOutlierRad = Math.max(0, maxRad);
    }

    // ===== Lifecycle =====
    public void resetAll() {
        xMeters = yMeters = headingRad = 0;
        initialized = false;
        bias.reset();
    }

    // ===== Odometry input (call every loop) =====
    public void updateOdometry(double x_m, double y_m, double heading_rad) {
        if (!initialized) {
            xMeters = x_m; yMeters = y_m; headingRad = wrap(heading_rad);
            initialized = true;
        } else {
            xMeters = x_m; yMeters = y_m; headingRad = wrap(heading_rad);
        }
    }

    // ===== Calibration controls =====
    /** Start/Restart the bias calibration window (use at the beginning of auton/teleop). */
    public void beginCalibration() { bias.reset(); }

    /** Feed one calibration sample; call while robot is still or creeping slowly. */
    public void feedCalibrationSample(double odoX, double odoY, double odoH, Pose3D llPose) {
        if (llPose == null || bias.locked) return;

        double xV = llPose.getPosition().x;
        double yV = llPose.getPosition().y;
        double hV = llPose.getOrientation().getYaw(AngleUnit.RADIANS);
        double dtx = odoX - xV;
        double dty = odoY - yV;
        double dth = wrap(odoH - hV);

        if (Math.hypot(dtx, dty) > calMaxTransOutlierM) return;
        if (Math.abs(dth) > calMaxYawOutlierRad) return;

        bias.add(dtx, dty, dth);
    }

    /** Lock calibration once enough good samples have been gathered. */
    public void endCalibration() { bias.locked = true; }

    public boolean isCalibrationLocked() { return bias.locked; }
    public BiasPose getBiasEstimate() { return new BiasPose(bias.dx, bias.dy, bias.dtheta); }

    // ===== Fusion (call whenever you have a LL frame to fuse) =====
    /** Fuses a Limelight pose (MT1 or MT2). Returns true if used; false if rejected. */
    public boolean fuseWithVision(Pose3D llPose, long stalenessMs) {
        if (!initialized || llPose == null) return false;
        if (stalenessMs > maxVisionStalenessMs) return false;

        // 1) Correct the vision pose by the learned bias (Vision ⊕ T_bias = Odom-aligned vision)
        Pose2d vCorr = applyBias(llPose); // may be identical if not locked (bias zero)

        // 2) Quality gates against current odometry
        final double dx = vCorr.x - xMeters;
        final double dy = vCorr.y - yMeters;
        final double dist = Math.hypot(dx, dy);
        if (dist > maxAcceptableJumpM) return false;

        final double dyaw = angDiff(vCorr.h, headingRad);
        if (Math.abs(dyaw) > maxAcceptableYawRad) return false;

        // 3) Age-scaled alpha
        final double a = alphaWithDecay(alphaVision, stalenessMs);

        // 4) Blend
        xMeters    = lerp(xMeters, vCorr.x, a);
        yMeters    = lerp(yMeters, vCorr.y, a);
        headingRad = wrap(headingRad + a * dyaw);
        return true;
    }

    // ===== Accessors =====
    public double getX() { return xMeters; }
    public double getY() { return yMeters; }
    public double getHeadingRad() { return headingRad; }
    public boolean isInitialized() { return initialized; }
    public Pose2d getPose2d() { return new Pose2d(xMeters, yMeters, headingRad); }

    // ===== Data structs =====
    public static class Pose2d { public final double x,y,h; public Pose2d(double x,double y,double h){this.x=x;this.y=y;this.h=h;} }
    public static class BiasPose { public final double dx,dy,dtheta; public BiasPose(double dx,double dy,double dtheta){this.dx=dx;this.dy=dy;this.dtheta=dtheta;} }

    // ===== Internal helpers =====
    private Pose2d applyBias(Pose3D llPose) {
        // Compose Vision ⊕ T_bias (rotate bias by current vision heading, then translate, then add yaw)
        double x = llPose.getPosition().x;
        double y = llPose.getPosition().y;
        double h = llPose.getOrientation().getYaw(AngleUnit.RADIANS);

        double xr = x + (bias.dx * Math.cos(h) - bias.dy * Math.sin(h));
        double yr = y + (bias.dx * Math.sin(h) + bias.dy * Math.cos(h));
        double hr = wrap(h + bias.dtheta);
        return new Pose2d(xr, yr, hr);
    }

    private static double lerp(double a, double b, double t) { return a + t * (b - a); }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private static double wrap(double a) {
        while (a <= -Math.PI) a += 2*Math.PI;
        while (a >   Math.PI) a -= 2*Math.PI;
        return a;
    }
    private static double angDiff(double target, double current) { return wrap(target - current); }

    private double alphaWithDecay(double baseAlpha, long ageMs) {
        if (ageMs <= 0) return baseAlpha;
        double factor = Math.pow(0.5, (double) ageMs / (double) halfLifeMs);
        double a = baseAlpha * factor;
        return clamp(a, 0.0, baseAlpha);
    }

    // ===== Bias estimator (running mean on dx,dy and circular mean on dθ) =====
    private static class BiasEstimator {
        double dx=0, dy=0, dtheta=0; boolean locked=false;
        int n=0; double sx=0, sy=0, sc=0, ss=0;
        void reset(){ dx=dy=dtheta=0; locked=false; n=0; sx=sy=sc=ss=0; }
        void add(double dtx, double dty, double dth){
            if (locked) return;
            sx += dtx; sy += dty;
            sc += Math.cos(dth); ss += Math.sin(dth);
            n++;
            dx = sx / n;
            dy = sy / n;
            dtheta = Math.atan2(ss / n, sc / n);
        }
    }
}

