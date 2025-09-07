package org.firstinspires.ftc.teamcode.config.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/**
 * PoseFusion_Subsystem
 * - Maintains a fused field pose using odometry + Limelight pose.
 * - Vision updates are exponentially blended with a configurable alpha.
 * - Rejects stale or "teleport" vision measurements.
 *
 * Coordinate system:
 *   - Positions in METERS (field frame).
 *   - Heading in RADIANS, wrapped to (-pi, pi].
 */
public class PoseFusion_subsystem {

    // ---- Tunables ----
    private double alphaVision = 0.20;     // 0..1 — how strongly to trust fresh vision
    private long   maxVisionStalenessMs = 120; // ignore LL frames older than this
    private double maxAcceptableJumpM   = 0.70; // reject vision if it disagrees with odo by more than this
    private double maxAcceptableYawRad  = Math.toRadians(25); // reject huge yaw jumps

    // Optional time decay: older vision gets less alpha automatically
    private long   halfLifeMs = 250;       // after this age, alpha halves (soft decay)

    // ---- State ----
    private double xMeters = 0, yMeters = 0, headingRad = 0;    // fused state
    private boolean initialized = false;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public PoseFusion_subsystem() {}

    // ---- Config setters ----
    public void setAlphaVision(double a) { alphaVision = clamp(a, 0, 1); }
    public void setMaxVisionStalenessMs(long ms) { maxVisionStalenessMs = Math.max(0, ms); }
    public void setMaxAcceptableJumpM(double m) { maxAcceptableJumpM = Math.max(0, m); }
    public void setMaxAcceptableYawRad(double r) { maxAcceptableYawRad = Math.max(0, r); }
    public void setHalfLifeMs(long ms) { halfLifeMs = Math.max(1, ms); }

    // ---- Odometry input (call every loop) ----
    /** Supply your current odometry estimate (Pedro/Pinpoint), meters + radians. */
    public void updateOdometry(double x_m, double y_m, double heading_rad) {
        if (!initialized) {
            xMeters = x_m; yMeters = y_m; headingRad = wrap(heading_rad);
            initialized = true;
        } else {
            xMeters = x_m; yMeters = y_m; headingRad = wrap(heading_rad);
        }
    }

    // ---- Vision input (call whenever you read Limelight) ----
    /**
     * Blend in a Limelight pose (MT1 or MT2) if valid and not stale.
     * @param llPose Pose3D from LL (result.getBotpose() or getBotpose_MT2()).
     * @param stalenessMs r.getStaleness() from the Limelight result.
     * @return true if fused, false if rejected.
     */
    public boolean fuseVision(Pose3D llPose, long stalenessMs) {
        if (!initialized || llPose == null) return false;
        if (stalenessMs > maxVisionStalenessMs) return false;

        // LL pose components (meters, radians)
        final double vx = llPose.getPosition().x;
        final double vy = llPose.getPosition().y;
        final double vyaw = wrap(llPose.getOrientation().getYaw(AngleUnit.RADIANS));

        // Reject if measurement is wildly inconsistent with odometry
        final double dx = vx - xMeters;
        final double dy = vy - yMeters;
        final double dist = Math.hypot(dx, dy);
        if (dist > maxAcceptableJumpM) return false;

        final double dyaw = angDiff(vyaw, headingRad);
        if (Math.abs(dyaw) > maxAcceptableYawRad) return false;

        // Compute age-scaled alpha (newer -> closer to alphaVision; older -> reduced)
        final double a = alphaWithDecay(alphaVision, stalenessMs);

        // Blend
        xMeters     = lerp(xMeters, vx, a);
        yMeters     = lerp(yMeters, vy, a);
        headingRad  = wrap(headingRad + a * dyaw);
        return true;
    }

    // ---- Accessors ----
    public double getX() { return xMeters; }
    public double getY() { return yMeters; }
    public double getHeadingRad() { return headingRad; }
    public boolean isInitialized() { return initialized; }

    // Convenience: return as simple struct
    public Pose2d getPose2d() { return new Pose2d(xMeters, yMeters, headingRad); }

    // ---- Helpers ----
    public static class Pose2d {
        public final double x, y, headingRad;
        public Pose2d(double x, double y, double h) { this.x=x; this.y=y; this.headingRad=h; }
    }

    private static double lerp(double a, double b, double t) { return a + t * (b - a); }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private static double wrap(double a) {
        // Wrap to (-pi, pi]
        while (a <= -Math.PI) a += 2*Math.PI;
        while (a >   Math.PI) a -= 2*Math.PI;
        return a;
    }

    private static double angDiff(double target, double current) {
        // Shortest signed angular difference target - current
        double d = wrap(target - current);
        return d;
    }

    private double alphaWithDecay(double baseAlpha, long ageMs) {
        if (ageMs <= 0) return baseAlpha;
        // Exponential half-life decay: alpha' = base * 0.5^(age/halfLife)
        double factor = Math.pow(0.5, (double) ageMs / (double) halfLifeMs);
        return clamp(baseAlpha * factor, 0.0, baseAlpha);
        // Newer frames ≈ baseAlpha; older frames fade toward 0.
    }
}

