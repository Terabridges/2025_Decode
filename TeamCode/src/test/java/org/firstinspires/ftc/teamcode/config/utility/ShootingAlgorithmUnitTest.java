package org.firstinspires.ftc.teamcode.config.utility;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShootingAlgorithmUnitTest {
    @Test
    public void testRegressionOnExampleData() throws Exception {
        ShooterTable t = ShooterTable.fromResource("/org/firstinspires/ftc/teamcode/config/utility/shooter_table_example.csv");
        assertNotNull(t);
        assertTrue(t.p1.length > 0);
        double d = 10.0;
        double g = 32.174;
        double thetaMin = Math.toRadians(45);
        double thetaMax = Math.toRadians(70);
        double vMin = 5.0;
        double vMax = 50.0;
        ShootingAlgorithm.ShotSolution sol = ShootingAlgorithm.findBestShotFromTable(d, g, t.p1, t.l1, thetaMin, thetaMax, vMin, vMax, 12, 12);
        assertNotNull(sol);
        assertTrue(sol.v >= vMin && sol.v <= vMax);
        assertTrue(sol.theta >= thetaMin && sol.theta <= thetaMax);
    }
}
