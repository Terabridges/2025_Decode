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
                // Load from packaged resource
                System.out.println("Loading example data from resource...");
                ShooterTable t = ShooterTable.fromResource("/org/firstinspires/ftc/teamcode/config/utility/shooter_table_example.csv");
                runRegressionAndPrint(t.p1, t.l1);
            } else {
                Path p = Paths.get(args[0]);
                System.out.println("Loading CSV from: " + p.toAbsolutePath());
                ShooterTable t = ShooterTable.fromFile(p);
                runRegressionAndPrint(t.p1, t.l1);
            }
        } catch (IOException ex) {
            System.err.println("Failed to load data: " + ex.getMessage());
            ex.printStackTrace();
        }
    }

    private static void runRegressionAndPrint(double[] p1, double[] l1) {
        if (p1 == null || p1.length == 0) {
            System.err.println("No data loaded");
            return;
        }
        // Parameters (example values)
        double d = 10.0;        // distance to front wall (same units as CSV)
        double g = 32.174;      // gravity (ft/s^2 if using feet)
        double thetaMin = Math.toRadians(45);
        double thetaMax = Math.toRadians(70);
        double vMin = 5.0;
        double vMax = 50.0;

        System.out.println("Data rows: " + p1.length);

        ShootingAlgorithm.ShotSolution sol = ShootingAlgorithm.findBestShotFromTable(
                d, g, p1, l1, thetaMin, thetaMax, vMin, vMax, 20, 20
        );

        if (sol != null) {
            System.out.printf("Optimal angle: %.3f degrees\n", Math.toDegrees(sol.theta));
            System.out.printf("Optimal velocity: %.3f\n", sol.v);
            System.out.printf("Estimated RPM: %.1f\n", ShootingAlgorithm.velocityToRPM(sol.v));
        } else {
            System.out.println("No feasible solution found");
        }
    }
}
