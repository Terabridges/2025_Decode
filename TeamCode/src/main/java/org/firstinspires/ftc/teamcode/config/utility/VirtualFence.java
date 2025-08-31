package org.firstinspires.ftc.teamcode.config.utility;// VirtualFence.java
// Drop this into TeamCode/src/main/java/your/package

import java.util.ArrayList;
import java.util.List;

public class VirtualFence
{
    public static class Vec2
    {
        public double x, y;

        public Vec2(double x, double y)
        {
            this.x = x;
            this.y = y;
        }

        public Vec2 add(Vec2 o)
        {
            return new Vec2(x + o.x, y + o.y);
        }

        public Vec2 sub(Vec2 o)
        {
            return new Vec2(x - o.x, y - o.y);
        }

        public Vec2 mul(double s)
        {
            return new Vec2(x * s, y * s);
        }

        public double dot(Vec2 o)
        {
            return x * o.x + y * o.y;
        }

        public double norm2()
        {
            return x * x + y * y;
        }

        public double norm()
        {
            return Math.sqrt(norm2());
        }
    }

    public static class Pose2
    {
        public double x, y, heading; // heading in radians, field frame

        public Pose2(double x, double y, double heading)
        {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /** Half-space wall: { x | n · x ≤ c } with outward normal n (unit or not) */
    public static class Wall
    {
        public Vec2 n;
        public double c;

        public Wall(Vec2 normal, double c)
        {
            this.n = normal;
            this.c = c;
        }

        /** Convenience: build from a line point p and outward normal n (not necessarily unit) */
        public static Wall fromPointNormal(Vec2 p, Vec2 n)
        {
            double c = n.dot(p);
            return new Wall(n, c);
        }

        /** Convenience: build from a line with direction tangent t (unit-ish) and a point p on the line.
         * The outward normal n is t rotated +90° or -90° depending on which side is "forbidden".
         */
        public static Wall fromPointTangent(Vec2 p, Vec2 t, boolean outwardLeftSide)
        {
            // Normal from tangent: left normal = (-t.y, t.x), right normal = (t.y, -t.x)
            Vec2 n = outwardLeftSide ? new Vec2(-t.y, t.x) : new Vec2(t.y, -t.x);
            double c = n.dot(p);
            return new Wall(n, c);
        }
    }

    /** Circular keep-out obstacle */
    public static class Bubble
    {
        public Vec2 center;
        public double radius; // already inflated by robot size

        public Bubble(Vec2 center, double radius)
        {
            this.center = center;
            this.radius = radius;
        }
    }

    private final List<Wall> walls = new ArrayList<>();
    private final List<Bubble> bubbles = new ArrayList<>();

    // Tuning parameters
    public double alpha = 4.0;          // Barrier aggressiveness (bigger = stronger pushback)
    public double alignDistance = 0.15; // meters to start auto-alignment behavior
    public double kPAng = 3.0;          // heading P gain (rad/s per rad)
    public boolean rideAlongWalls = true;

    // For auto-alignment: choose whether to align parallel or perpendicular to a wall
    public enum AlignMode { PARALLEL, PERPENDICULAR }
    public AlignMode alignMode = AlignMode.PARALLEL;

    // Adders
    public void addWall(Wall w)
    {
        walls.add(w);
    }

    public void addBubble(Bubble b)
    {
        bubbles.add(b);
    }

    /** Filter a desired planar velocity (field frame) to be safe under walls + bubbles. */
    public Vec2 filterVelocity(Vec2 vDes, Pose2 pose)
    {
        Vec2 x = new Vec2(pose.x, pose.y);
        Vec2 v = new Vec2(vDes.x, vDes.y);

        // 1) Enforce half-space (wall) constraints via projection if needed
        for (Wall w : walls)
        {
            double nx = w.n.x, ny = w.n.y;
            double nNorm2 = nx * nx + ny * ny;
            if (nNorm2 < 1e-9)
            {
                continue;
            }

            // Signed distance to wall along normal: d = c - n · x  (positive inside the safe region)
            double d = w.c - (w.n.dot(x));

            // Barrier inequality on velocity: n · v ≤ alpha * d
            double rhs = alpha * d;
            double lhs = w.n.dot(v);

            if (lhs > rhs)
            {
                // Project v onto the half-space boundary: v <- v - ((lhs - rhs)/||n||^2) * n
                double scale = (lhs - rhs) / nNorm2;
                v = v.sub(w.n.mul(scale));
            }
        }

        // 2) Enforce bubble keep-outs (h = ||x - p0||^2 - R^2 ≥ 0;  ∇h = 2(x - p0))
        for (Bubble b : bubbles)
        {
            Vec2 r = x.sub(b.center); // from bubble center to robot
            double h = r.norm2() - b.radius * b.radius; // should be ≥ 0
            Vec2 gradH = r.mul(2.0);

            double lhs = gradH.dot(v);
            double rhs = -alpha * h; // ∇h · v ≥ -α h

            if (lhs < rhs)
            {
                double gradNorm2 = gradH.norm2();
                if (gradNorm2 > 1e-9)
                {
                    // Project v to satisfy ∇h · v = rhs
                    double delta = (rhs - lhs) / gradNorm2;
                    v = v.add(gradH.mul(delta));
                }
            }
        }

        return v;
    }

    /** Optional: compute an angular velocity command that aligns to the nearest wall if close. */
    public double alignmentOmega(Pose2 pose)
    {
        if (walls.isEmpty())
        {
            return 0.0;
        }

        // Find "nearest" wall by signed normal distance
        Wall best = null;
        double bestAbsD = Double.POSITIVE_INFINITY;
        Vec2 x = new Vec2(pose.x, pose.y);

        for (Wall w : walls)
        {
            double d = (w.c - w.n.dot(x)) / Math.max(1e-9, Math.sqrt(w.n.norm2())); // meters
            double ad = Math.abs(d);
            if (ad < bestAbsD)
            {
                bestAbsD = ad;
                best = w;
            }
        }

        if (best == null || bestAbsD > alignDistance)
        {
            return 0.0; // too far to bother aligning
        }

        // Desired heading
        double thetaDes;
        // Tangent direction of wall = rotate normal by -90° (t = (ny, -nx)) OR +90°. We just need an angle.
        Vec2 n = best.n;
        Vec2 t = new Vec2(n.y, -n.x);

        if (alignMode == AlignMode.PARALLEL)
        {
            thetaDes = Math.atan2(t.y, t.x);
        }
        else
        {
            thetaDes = Math.atan2(n.y, n.x);
        }

        // Smallest angle error
        double err = wrapAngle(thetaDes - pose.heading);
        return kPAng * err; // rad/s
    }

    /** If riding along walls, project v onto wall tangent when near that wall. */
    public Vec2 rideAlongIfClose(Vec2 v, Pose2 pose)
    {
        if (!rideAlongWalls || walls.isEmpty())
        {
            return v;
        }

        Wall best = null;
        double bestAbsD = Double.POSITIVE_INFINITY;
        Vec2 x = new Vec2(pose.x, pose.y);

        for (Wall w : walls)
        {
            double d = (w.c - w.n.dot(x)) / Math.max(1e-9, Math.sqrt(w.n.norm2()));
            double ad = Math.abs(d);
            if (ad < bestAbsD)
            {
                bestAbsD = ad;
                best = w;
            }
        }

        if (best == null || bestAbsD > alignDistance)
        {
            return v;
        }

        // Project onto tangent: v_parallel = v - proj_n(v)
        Vec2 n = best.n;
        double nNorm2 = n.norm2();
        if (nNorm2 < 1e-9)
        {
            return v;
        }
        double coeff = v.dot(n) / nNorm2;
        return v.sub(n.mul(coeff));
    }

    private static double wrapAngle(double a)
    {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
