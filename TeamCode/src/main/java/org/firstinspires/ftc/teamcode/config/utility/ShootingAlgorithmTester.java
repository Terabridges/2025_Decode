package org.firstinspires.ftc.teamcode.config.utility;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Simple tester that can be run from the command line.
 * Usage:
 *  - No args: loads the bundled example CSV resource
 *  - One arg: path to CSV file to load
 */
public class ShootingAlgorithmTester {
    public static void main(String[] args) {
        try {
            if (args.length == 0) {
                // Test with exact Desmos example
                System.out.println("Testing with Desmos example values...");
                double[] p1 = {-1.4, 0.1};
                double[] l1 = {3.23, 4.13};
                double d = 8.6;
                runRegressionAndPrint(p1, l1, d);
                
                System.out.println("\n" + "=".repeat(60) + "\n");
                
                // Load from packaged resource
                System.out.println("Loading example data from resource...");
                ShooterTable t = ShooterTable.fromResource("/org/firstinspires/ftc/teamcode/config/utility/shooter_table_example.csv");
                runRegressionAndPrint(t.p1, t.l1, 10.0);
            } else {
                Path p = Paths.get(args[0]);
                System.out.println("Loading CSV from: " + p.toAbsolutePath());
                ShooterTable t = ShooterTable.fromFile(p);
                runRegressionAndPrint(t.p1, t.l1, 10.0);
            }
        } catch (IOException ex) {
            System.err.println("Failed to load data: " + ex.getMessage());
            ex.printStackTrace();
        }
    }

    private static void runRegressionAndPrint(double[] p1, double[] l1, double d) {
        if (p1 == null || p1.length == 0) {
            System.err.println("No data loaded");
            return;
        }
        // Parameters
        double g = 32.174;      // gravity (ft/s^2)
        double thetaMin = Math.toRadians(45);
        double thetaMax = Math.toRadians(70);
        double vMin = 5.0;
        double vMax = 50.0;

        System.out.println("Distance to goal (d): " + d + " ft");
        System.out.println("Data rows: " + p1.length);
        System.out.print("p1 values: [");
        for (int i = 0; i < p1.length; i++) {
            System.out.print(p1[i]);
            if (i < p1.length - 1) System.out.print(", ");
        }
        System.out.println("]");
        System.out.print("l1 values: [");
        for (int i = 0; i < l1.length; i++) {
            System.out.print(l1[i]);
            if (i < l1.length - 1) System.out.print(", ");
        }
        System.out.println("]");

        ShootingAlgorithm.ShotSolution sol = ShootingAlgorithm.findBestShotFromTable(
                d, g, p1, l1, thetaMin, thetaMax, vMin, vMax, 20, 20
        );

        if (sol != null) {
            System.out.println("\n--- Results ---");
            System.out.printf("Optimal angle (s_opt): %.5f degrees\n", Math.toDegrees(sol.theta));
            System.out.printf("Optimal velocity (v_opt): %.5f ft/s\n", sol.v);
            System.out.printf("Estimated RPM: %.1f\n", ShootingAlgorithm.velocityToRPM(sol.v));
            System.out.printf("R² value: %.12f\n", sol.rSquared);
            System.out.printf("RSS (residual sum of squares): %.12f\n", sol.rss);
            System.out.printf("TSS (total sum of squares): %.12f\n", sol.tss);
            
            if (sol.rSquared >= 0.999) {
                System.out.println("✓ Excellent fit (R² ≥ 0.999)");
            } else if (sol.rSquared >= 0.99) {
                System.out.println("✓ Good fit (R² ≥ 0.99)");
            } else {
                System.out.println("⚠ Warning: Fit quality below target (R² < 0.99)");
            }
        } else {
            System.out.println("No feasible solution found");
        }
    }
}