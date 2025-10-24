package org.firstinspires.ftc.teamcode.utility;

public class ShotAlgorithm {

    private double thetaMin = 0;
    private double thetaMax = 195;

    private double vMax = 25;

    public ShotAlgorithm(double thetaMin, double thetaMax, double vMax) {
        this.thetaMin = thetaMin;
        this.thetaMax = thetaMax;
        this.vMax = vMax;
    }



    // Utils: solve quadratic ax^2 + bx + c = 0
    private static double[] solveQuadratic(double a, double b, double c) {
        if (Math.abs(a) < 1e-12) {
            if (Math.abs(b) < 1e-12) return new double[0];
            return new double[]{ -c / b }; // linear
        }
        double disc = b*b - 4*a*c;
        if (disc < 0) return new double[0];
        double sqrtD = Math.sqrt(disc);
        double q = (b > 0) ? -0.5 * (b + sqrtD) : -0.5 * (b - sqrtD);
        double r1 = q / a;
        double r2 = c / q;
        return new double[]{ r1, r2 };
    }

    // datatype to store best (theta, v)
    public static class ShotSolution {
        public final double theta; // radians
        public final double v;     // muzzle speed (m/s)

        public ShotSolution(double theta, double v) {
            this.theta = theta; this.v = v;
        }
    }

    // Find best (theta,v) by scanning then refining
    public ShotSolution findBestShot(
            double x, double y,      // target offsets (m): x horizontal dist, y vertical diff goal - launcher
            double ux, double uy,    // robot velocity components (m/s)
            double thetaMin, double thetaMax, // radians
            double vMax,             // max muzzle speed (m/s)
            int steps                // grid steps for initial scan
    ) {
        ShotSolution best = null;
        for (int i = 0; i <= steps; i++) {
            double t = (double)i / steps;
            double theta = thetaMin + t * (thetaMax - thetaMin);
            double A = Math.cos(theta), B = Math.sin(theta);
            // coefficients
            double a = x * A * B - y * A * A;
            double b = x * (A * uy + B * ux) - 2 * y * A * ux;
            double c = x * ux * uy - y * ux * ux - 0.5 * 9.81 * x * x;
            double[] roots = solveQuadratic(a, b, c);
            for (double vCandidate : roots) {
                if (Double.isNaN(vCandidate) || vCandidate <= 0) continue;
                double Vx0 = vCandidate * A + ux;
                if (Vx0 <= 0) continue; // invalid: won't reach target forward
                if (vCandidate > vMax) continue; // beyond flywheel capability
                if (best == null || vCandidate < best.v) {
                    best = new ShotSolution(theta, vCandidate);
                }
            }
        }
        // optional: refine best.theta with local golden section / more steps
        if (best != null) {
            // simple refinement: do denser scan around best.theta
            double span = Math.max( (thetaMax - thetaMin) * 0.02, Math.toRadians(1.0) );
            double left = Math.max(thetaMin, best.theta - span);
            double right = Math.min(thetaMax, best.theta + span);
            for (int i=0;i<=200;i++) {
                double theta = left + (right-left) * ((double)i/200.0);
                double A = Math.cos(theta), B = Math.sin(theta);
                double a = x * A * B - y * A * A;
                double b = x * (A * uy + B * ux) - 2 * y * A * ux;
                double c = x * ux * uy - y * ux * ux - 0.5 * 9.81 * x * x;
                double[] roots = solveQuadratic(a, b, c);
                for (double vCandidate : roots) {
                    if (Double.isNaN(vCandidate) || vCandidate <= 0) continue;
                    double Vx0 = vCandidate * A + ux;
                    if (Vx0 <= 0) continue;
                    if (vCandidate > vMax) continue;
                    if (vCandidate < best.v) best = new ShotSolution(theta, vCandidate);
                }
            }
        }
        return best; // may be null if no feasible solution
    }

    // Example mapping between muzzle speed and flywheel RPM (calibrate this)
    public double velocityToRPM(double v) {
        // Example linear mapping from calibration: v = k*RPM + b
        // So RPM = (v - b) / k
        double k = 0.003507; // m/s per RPM, example - calibrate!
        double b = 0.0;
        return (v - b) / k;
    }

}


/* Example Usage:

double x = measuredDistanceToGoal;   // m
double y = goalHeight - launcherHeight; // m
double ux = robotVelX; // m/s (positive toward goal)
double uy = 0;
double thetaMin = Math.toRadians(10);
double thetaMax = Math.toRadians(70);
double vMax = 25.0; // m/s (example)
ShotSolution sol = findBestShot(x,y,ux,uy,thetaMin,thetaMax,vMax, 200);
if (sol != null) {
double targetRPM = velocityToRPM(sol.v);
    turretController.moveToAngle(sol.theta);
    flywheelController.setTargetRPM(targetRPM);
} else {
        // no feasible shot in constraints
        }

 */