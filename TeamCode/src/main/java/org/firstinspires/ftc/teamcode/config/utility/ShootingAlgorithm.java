package org.firstinspires.ftc.teamcode.config.utility;

import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer.Optimum;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.util.Pair;

public class ShootingAlgorithm {

    private double thetaMin;
    private double thetaMax;

    private double vMax;

    public ShootingAlgorithm(double thetaMin, double thetaMax, double vMax) {
        this.thetaMin = thetaMin;
        this.thetaMax = thetaMax;
        this.vMax = vMax;
    }

    // datatype to store best (theta, v)
    public static class ShotSolution {
        public final double theta; // radians
        public final double v;     // muzzle speed (units consistent with data)

        public ShotSolution(double theta, double v) {
            this.theta = theta; this.v = v;
        }
    }

    // --- Regression-based solver -------------------------------------------------
    // Use the Desmos model
    //   l1 = -g/2 * ((d - p1) / (v * cos(s)))^2 + v * sin(s) * ((d - p1) / (v * cos(s))) + u
    // where s in radians, v positive. We fit (s, v) to a provided dataset {p1_i, l1_i}
    // using a coarse grid search followed by Levenberg-Marquardt refinement.

    /**
     * Find best shot parameters (theta, v) by fitting the regression model to the
     * provided table of (p1, l1) points for a given launcher distance d and gravity g.
     *
     * Units: ensure d, p1, l1 and g are in consistent units (e.g., feet and ft/s^2, or meters and m/s^2).
     *
     * @param d distance from launcher to front wall of goal (same units as p1)
     * @param g gravity (positive scalar, same length/time^2 units as p1 and l1)
     * @param p1 array of x-values from table (same units as d)
     * @param l1 array of y-values from table (same units as d/p1)
     * @param thetaMin lower bound for angle (radians)
     * @param thetaMax upper bound for angle (radians)
     * @param vMin lower bound for speed (must be > 0)
     * @param vMax upper bound for speed
     * @param gridThetaSteps coarse grid steps for angle
     * @param gridVSteps coarse grid steps for speed
     * @return ShotSolution or null if inputs invalid / no feasible solution
     */
    public static ShotSolution findBestShotFromTable(
            final double d, final double g,
            final double[] p1, final double[] l1,
            final double thetaMin, final double thetaMax,
            final double vMin, final double vMax,
            final int gridThetaSteps, final int gridVSteps
    ) {
        if (p1 == null || l1 == null) return null;
        if (p1.length != l1.length || p1.length < 1) return null;
        final int n = p1.length;

        // Coarse grid search to get a robust initial guess
        double bestS = thetaMin;
        double bestV = Math.max(vMin, Math.min(vMax, (vMin + vMax) / 2.0));
        double bestError = Double.POSITIVE_INFINITY;
        for (int i = 0; i <= gridThetaSteps; i++) {
            double t = (double)i / (double)gridThetaSteps;
            double s = thetaMin + t * (thetaMax - thetaMin);
            for (int j = 0; j <= gridVSteps; j++) {
                double u = (double)j / (double)gridVSteps;
                double v = vMin + u * (vMax - vMin);
                double err = 0.0;
                for (int k = 0; k < n; k++) {
                    double pred = predictY(d, p1[k], s, v, g);
                    double r = pred - l1[k];
                    err += r*r;
                }
                if (err < bestError) {
                    bestError = err; bestS = s; bestV = v;
                }
            }
        }

        // Local refinement using Levenberg-Marquardt
        final double[] target = new double[n];
        for (int i = 0; i < n; i++) target[i] = l1[i];

        MultivariateJacobianFunction model = new MultivariateJacobianFunction() {
            @Override
            public org.apache.commons.math3.util.Pair<org.apache.commons.math3.linear.RealVector, org.apache.commons.math3.linear.RealMatrix> value(org.apache.commons.math3.linear.RealVector point) {
                double s = point.getEntry(0);
                double v = point.getEntry(1);
                double[] value = new double[n];
                double[][] jac = new double[n][2];
                for (int i = 0; i < n; i++) {
                    double pi = p1[i];
                    double pred = predictY(d, pi, s, v, g);
                    value[i] = pred;
                    // partials
                    double[] jw = jacobianY(d, pi, s, v, g);
                    jac[i][0] = jw[0]; // d/ds
                    jac[i][1] = jw[1]; // d/dv
                }
                RealVector valVec = new ArrayRealVector(value);
                RealMatrix jacMat = new Array2DRowRealMatrix(jac);
                return org.apache.commons.math3.util.Pair.create(valVec, jacMat);
            }
        };

        LeastSquaresBuilder builder = new LeastSquaresBuilder()
                .start(new double[]{bestS, bestV})
                .model(model)
                .target(new ArrayRealVector(target))
                .maxEvaluations(2000)
                .maxIterations(2000);

        Optimum optimum = null;
        try {
            Optimum opt = new LevenbergMarquardtOptimizer().optimize(builder.build());
            optimum = opt;
        } catch (Exception ex) {
            // optimizer failed — we'll fall back to coarse-grid best
            optimum = null;
        }

        if (optimum != null) {
            double sOpt = optimum.getPoint().getEntry(0);
            double vOpt = optimum.getPoint().getEntry(1);
            // enforce bounds
            if (sOpt < thetaMin) sOpt = thetaMin;
            if (sOpt > thetaMax) sOpt = thetaMax;
            if (vOpt < vMin) vOpt = vMin;
            if (vOpt > vMax) vOpt = vMax;
            return new ShotSolution(sOpt, vOpt);
        } else {
            // coarse grid fallback
            return new ShotSolution(bestS, bestV);
        }
    }

    // predict y using the regression model for a single sample
    private static double predictY(double d, double p1, double s, double v, double g) {
        double delta = d - p1;
        double cosS = Math.cos(s);
        double sinS = Math.sin(s);
        if (v <= 0 || Math.abs(cosS) < 1e-12) return Double.NaN;
        double t = delta / (v * cosS);
        return -0.5 * g * t * t + v * sinS * t;
    }

    // jacobian: returns [d/ds, d/dv]
    private static double[] jacobianY(double d, double p1, double s, double v, double g) {
        double delta = d - p1;
        double cosS = Math.cos(s);
        double sinS = Math.sin(s);
        if (v <= 0 || Math.abs(cosS) < 1e-12) return new double[]{0.0, 0.0};
        double t = delta / (v * cosS);
        // dt/dv = -t / v
        double dt_dv = -t / v;
        // dt/ds = t * tan(s)
        double dt_ds = t * Math.tan(s);

        // pred = -g/2 * t^2 + v*sin(s) * t
        // d/dv: -g * t * dt_dv + sin(s)*t + v*sin(s)*dt_dv
        double d_dv = -g * t * dt_dv + sinS * t + v * sinS * dt_dv;
        // Simplify: often reduces, but keep explicit

        // d/ds: -g * t * dt_ds + v*cos(s)*t + v*sin(s)*dt_ds
        double d_ds = -g * t * dt_ds + v * cosS * t + v * sinS * dt_ds;

        return new double[]{d_ds, d_dv};
    }

    /**
     * Convert velocity (ft/s) to RPM using a linear calibration.
     * Assumes a linear relationship: RPM = (v * 60) / (2 * π * r)
     * where r is the radius of the wheel in feet.
     * @param v velocity in ft/s
     * @param r radius of the wheel in feet
     * */
    public static double velocityToRPM(double v, double r) {
        if (r <= 0) return 0.0;
        return (v * 60.0) / (2.0 * Math.PI * r);
    }

    /*
     Example usage:
        // Suppose you have data table arrays p1 and l1 loaded from CSV (units consistent with d and g)
        double[] p1 = {1.0, 2.0, 3.0};
        double[] l1 = {2.4, 2.1, 1.8};
        double d = 10.0;        // distance to front wall (same units)
        double g = 32.174;      // ft/s^2 if using feet
        ShotSolution sol = findBestShotFromTable(d, g, p1, l1, Math.toRadians(45), Math.toRadians(70), 5.0, 50.0, 20, 20);
        if (sol != null) {
            // sol.theta (radians), sol.v (units consistent with data)
        }
    */

}