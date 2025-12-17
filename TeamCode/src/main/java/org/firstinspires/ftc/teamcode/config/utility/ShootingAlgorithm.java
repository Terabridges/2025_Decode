package org.firstinspires.ftc.teamcode.config.utility;

import org.apache.commons.math3.fitting.leastsquares.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.optim.SimpleValueChecker;
import org.apache.commons.math3.util.Pair;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;

public class ShootingAlgorithm {

    public static class ShotSolution {
        public final double theta;
        public final double v;
        public final double rSquared;  // NEW: R² value
        public final double rss;        // NEW: residual sum of squares
        public final double tss;        // NEW: total sum of squares

        public ShotSolution(double theta, double v, double rSquared, double rss, double tss) {
            this.theta = theta;
            this.v = v;
            this.rSquared = rSquared;
            this.rss = rss;
            this.tss = tss;
        }

        // Backward compatibility
        public ShotSolution(double theta, double v) {
            this(theta, v, Double.NaN, Double.NaN, Double.NaN);
        }
    }

    /**
     * Enhanced regression-based shot solver with multiple strategies and R² calculation.
     * 
     * @param d distance from launcher to goal front wall
     * @param g gravity constant
     * @param p1 array of x positions (relative to goal)
     * @param l1 array of y positions (heights)
     * @param thetaMin minimum angle (radians)
     * @param thetaMax maximum angle (radians)
     * @param vMin minimum velocity
     * @param vMax maximum velocity
     * @param gridStepsTheta grid resolution for theta (increased default recommended)
     * @param gridStepsV grid resolution for velocity (increased default recommended)
     * @return ShotSolution with theta, v, and R² value
     */
    public static ShotSolution findBestShotFromTable(
            double d, double g, double[] p1, double[] l1,
            double thetaMin, double thetaMax, double vMin, double vMax,
            int gridStepsTheta, int gridStepsV) {

        if (p1 == null || l1 == null || p1.length != l1.length || p1.length == 0) {
            return null;
        }

        int n = p1.length;

        // Calculate mean for R² computation
        double meanL1 = 0.0;
        for (double val : l1) {
            meanL1 += val;
        }
        meanL1 /= n;

        // Strategy 1: Fine grid search (increased resolution)
        double bestTheta = Double.NaN;
        double bestV = Double.NaN;
        double bestRSS = Double.POSITIVE_INFINITY;

        int fineGridTheta = Math.max(gridStepsTheta * 5, 100); // 5x finer or at least 100
        int fineGridV = Math.max(gridStepsV * 5, 100);

        for (int iTheta = 0; iTheta <= fineGridTheta; iTheta++) {
            double theta = thetaMin + iTheta * (thetaMax - thetaMin) / fineGridTheta;
            for (int iV = 0; iV <= fineGridV; iV++) {
                double v = vMin + iV * (vMax - vMin) / fineGridV;

                double rss = 0.0;
                boolean valid = true;
                for (int i = 0; i < n; i++) {
                    double pred = predictY(d, g, p1[i], theta, v);
                    if (Double.isNaN(pred) || Double.isInfinite(pred)) {
                        valid = false;
                        break;
                    }
                    double residual = l1[i] - pred;
                    rss += residual * residual;
                }

                if (valid && rss < bestRSS) {
                    bestRSS = rss;
                    bestTheta = theta;
                    bestV = v;
                }
            }
        }

        if (Double.isNaN(bestTheta)) {
            return null; // No valid solution found in grid
        }

        // Strategy 2: Refine with Nelder-Mead simplex (derivative-free, robust)
        final double finalD = d;
        final double finalG = g;
        final double finalMeanL1 = meanL1;

        MultivariateFunction objectiveFunction = params -> {
            double theta = params[0];
            double v = params[1];

            // Enforce bounds
            if (theta < thetaMin || theta > thetaMax || v < vMin || v > vMax) {
                return Double.POSITIVE_INFINITY;
            }

            double rss = 0.0;
            for (int i = 0; i < n; i++) {
                double pred = predictY(finalD, finalG, p1[i], theta, v);
                if (Double.isNaN(pred) || Double.isInfinite(pred)) {
                    return Double.POSITIVE_INFINITY;
                }
                double residual = l1[i] - pred;
                rss += residual * residual;
            }
            return rss;
        };

        try {
            SimplexOptimizer optimizer = new SimplexOptimizer(1e-12, 1e-12); // Very tight tolerances
            PointValuePair result = optimizer.optimize(
                    new MaxEval(10000),
                    new ObjectiveFunction(objectiveFunction),
                    GoalType.MINIMIZE,
                    new InitialGuess(new double[]{bestTheta, bestV}),
                    new NelderMeadSimplex(2, 0.001) // Small initial simplex
            );

            double[] optimized = result.getPoint();
            bestTheta = optimized[0];
            bestV = optimized[1];
            bestRSS = result.getValue();

        } catch (Exception e) {
            // If Nelder-Mead fails, use grid search result
            System.err.println("Nelder-Mead refinement failed, using grid search result: " + e.getMessage());
        }

        // Strategy 3: Final polish with Levenberg-Marquardt
        try {
            LeastSquaresProblem lsp = new LeastSquaresBuilder()
                    .start(new double[]{bestTheta, bestV})
                    .model(new MultivariateJacobianFunction() {
                        @Override
                        public Pair<RealVector, RealMatrix> value(RealVector point) {
                            double theta = point.getEntry(0);
                            double v = point.getEntry(1);
                            return jacobianY(finalD, finalG, p1, l1, theta, v);
                        }
                    })
                    .target(l1)
                    .lazyEvaluation(false)
                    .maxEvaluations(10000)
                    .maxIterations(10000)
                    .checker(new SimpleValueChecker(1e-12, 1e-12)) // Very tight tolerances
                    .build();

            LeastSquaresOptimizer.Optimum optimum = new LevenbergMarquardtOptimizer().optimize(lsp);
            RealVector sol = optimum.getPoint();
            double finalTheta = sol.getEntry(0);
            double finalV = sol.getEntry(1);

            // Validate bounds
            if (finalTheta >= thetaMin && finalTheta <= thetaMax && finalV >= vMin && finalV <= vMax) {
                bestTheta = finalTheta;
                bestV = finalV;
                bestRSS = optimum.getRMS() * optimum.getRMS() * n; // RMS to RSS
            }
        } catch (Exception e) {
            // If LM fails, use Nelder-Mead/grid result
            System.err.println("LM refinement failed, using previous result: " + e.getMessage());
        }

        // Calculate R²
        double tss = 0.0;
        for (double val : l1) {
            double diff = val - meanL1;
            tss += diff * diff;
        }

        double rSquared = (tss > 0) ? 1.0 - (bestRSS / tss) : 1.0;

        return new ShotSolution(bestTheta, bestV, rSquared, bestRSS, tss);
    }

    /**
     * Predict y (height) given parameters.
     */
    private static double predictY(double d, double g, double p1Val, double theta, double v) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        if (Math.abs(cosTheta) < 1e-12 || v < 1e-12) {
            return Double.NaN;
        }
        double dx = d - p1Val;
        double t = dx / (v * cosTheta);
        if (t < 0) {
            return Double.NaN;
        }
        return -0.5 * g * t * t + v * sinTheta * t;
    }

    /**
     * Compute residuals and Jacobian matrix for LM.
     */
    private static Pair<RealVector, RealMatrix> jacobianY(
            double d, double g, double[] p1, double[] l1, double theta, double v) {

        int n = p1.length;
        double[] residuals = new double[n];
        double[][] jac = new double[n][2];

        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        double cos2 = cosTheta * cosTheta;
        double vCos = v * cosTheta;

        for (int i = 0; i < n; i++) {
            double dx = d - p1[i];
            if (Math.abs(vCos) < 1e-12) {
                residuals[i] = 0;
                jac[i][0] = 0;
                jac[i][1] = 0;
                continue;
            }

            double t = dx / vCos;
            double pred = -0.5 * g * t * t + v * sinTheta * t;
            residuals[i] = l1[i] - pred;

            // Partial derivatives
            double dt_dtheta = dx * sinTheta / (v * cos2);
            double dt_dv = -dx / (v * v * cosTheta);

            double dPred_dtheta = -g * t * dt_dtheta + v * Math.cos(theta) * t + v * sinTheta * dt_dtheta;
            double dPred_dv = -g * t * dt_dv + sinTheta * t + v * sinTheta * dt_dv;

            jac[i][0] = -dPred_dtheta; // negative because residual = observed - predicted
            jac[i][1] = -dPred_dv;
        }

        return Pair.create(MatrixUtils.createRealVector(residuals), MatrixUtils.createRealMatrix(jac));
    }

    /**
     * Convert velocity (ft/s) to RPM.
     */
    public static double velocityToRPM(double velocity) {
        double wheelDiameter = 4.0 / 12.0; // 4 inches in feet
        double wheelCircumference = Math.PI * wheelDiameter;
        return (velocity / wheelCircumference) * 60.0;
    }
}