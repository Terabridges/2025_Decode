package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utility.ShotAlgorithm;
import org.firstinspires.ftc.teamcode.utility.ShotAlgorithm.ShotSolution;

import java.util.ArrayList;
import java.util.List;

/**
 * ShotAlgorithmCalibrator
 *
 * Purpose: Calibrate the linear map between flywheel RPM and muzzle speed v (m/s):
 *              v = k * RPM + b
 * using the original ShotAlgorithm to compute required v* for a given geometry and angle bounds.
 *
 * Workflow:
 *  1) Set geometry x (m), y (m), and angle bounds (θmin/θmax in deg).
 *  2) Spin your wheels to an RPM, shoot, and when it SCORES press A to RECORD.
 *     The code logs (avgMeasuredRPM, required_v_from_ShotAlgorithm).
 *  3) Collect ~8–12 samples across your range. Watch k, b, and R² stabilize.
 *  4) Use k and b in your code (e.g., velocityToRPM and its inverse).
 */
@Configurable
@TeleOp(name = "ShotAlgorithmCalibrator", group = "Test")
public class RPMCalibration extends LinearOpMode
{
    // Hardware
    public Shooter shooter;
    public Vision vision;
    private VoltageSensor battery;

    // Telemetry
    private JoinedTelemetry joinedTelemetry;

    // Gamepad edge detection
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    // Selection: 0=stop, 1=left, 2=right, 3=both
    public int selectedRunType = 0;

    // ======= Tunables (Dashboard) =======
    public static double xMeters = 3.00;        // horizontal distance to goal (+forward)
    public static double yMeters = 0.45;        // goalHeight - launcherHeight
    public static double thetaMinDeg = 10.0;    // hood lower bound
    public static double thetaMaxDeg = 70.0;    // hood upper bound
    public static int steps = 200;              // coarse scan steps for ShotAlgorithm
    public static double vMaxMps = 30.0;        // safe upper bound for v in solver (does not affect RPM cap)

    // Robot velocity (for calibration, keep 0)
    public static double ux = 0.0;
    public static double uy = 0.0;

    // Wheel control
    public static double targetRPM = 3000.0;
    public static double rpmStep = 50.0;

    // Hood hard limits
    private static final double HOOD_MIN = 0.0, HOOD_MAX = 195.0;

    // Motor ticks<->RPM (match your Velocity tester)
    private final double ticksPerRev = 28.0;

    // Samples: (RPM_measured, v_required)
    private final List<Double> rpmSamples = new ArrayList<>();
    private final List<Double> vSamples = new ArrayList<>();

    // Fit results
    private double fitK = Double.NaN; // m/s per RPM
    private double fitB = Double.NaN; // m/s offset
    private double fitR2 = Double.NaN;

    @Override
    public void runOpMode()
    {
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        vision = new Vision(hardwareMap);
        shooter = new Shooter(hardwareMap, vision);
        battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive())
        {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            handleModeButtons();
            handleAdjustments();

            // Compute required v* from original ShotAlgorithm
            ShotSolution sol = computeRequiredV();

            // Spin wheels to target RPM; user aims/fires manually
            applyShooter(targetRPM);

            // Record a data point when operator confirms a score (press A)
            if (pressed(currentGamepad1.a, previousGamepad1.a))
            {
                recordSample(sol);
            }

            // Back clears samples; Start saves JSON
            if (pressed(currentGamepad1.back, previousGamepad1.back))
            {
                clearSamples();
            }
            if (pressed(currentGamepad1.start, previousGamepad1.start))
            {
                saveCalibrationJson();
            }

            updateTelem(sol);
        }
    }

    private void handleModeButtons()
    {
        if (pressed(currentGamepad1.x, previousGamepad1.x)) selectedRunType = 1;
        if (pressed(currentGamepad1.b, previousGamepad1.b)) selectedRunType = 2;
        if (pressed(currentGamepad1.y, previousGamepad1.y)) selectedRunType = 3;
        if (pressed(currentGamepad1.a, previousGamepad1.a)) selectedRunType = 0; // also used to record; that's okay
    }

    private void handleAdjustments()
    {
        if (pressed(currentGamepad1.dpad_left,  previousGamepad1.dpad_left))  xMeters = Math.max(0.1, xMeters - 0.05);
        if (pressed(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) xMeters += 0.05;
        if (pressed(currentGamepad1.dpad_down,  previousGamepad1.dpad_down))  yMeters -= 0.02;
        if (pressed(currentGamepad1.dpad_up,    previousGamepad1.dpad_up))    yMeters += 0.02;

        if (pressed(currentGamepad1.left_stick_button,  previousGamepad1.left_stick_button))  thetaMinDeg = Math.max(HOOD_MIN, thetaMinDeg - 0.5);
        if (pressed(currentGamepad1.right_stick_button, previousGamepad1.right_stick_button)) thetaMaxDeg = Math.min(HOOD_MAX, thetaMaxDeg + 0.5);
        if (thetaMaxDeg < thetaMinDeg) thetaMaxDeg = thetaMinDeg;

        if (pressed(currentGamepad1.left_bumper,  previousGamepad1.left_bumper))  targetRPM = Math.max(0, targetRPM - rpmStep);
        if (pressed(currentGamepad1.right_bumper, previousGamepad1.right_bumper)) targetRPM += rpmStep;
    }

    private ShotSolution computeRequiredV()
    {
        double tMin = Math.toRadians(clamp(thetaMinDeg, HOOD_MIN, HOOD_MAX));
        double tMax = Math.toRadians(clamp(thetaMaxDeg, HOOD_MIN, HOOD_MAX));

        ShotAlgorithm alg = new ShotAlgorithm(tMin, tMax, vMaxMps);
        return alg.findBestShot(
                xMeters, yMeters,
                ux, uy,
                tMin, tMax,
                vMaxMps,
                Math.max(steps, 50)
        );
    }

    private void applyShooter(double rpm)
    {
        double velTicks = RPMToVel(rpm);

        if (selectedRunType == 1)
        {
            shooter.flyLeft.setVelocity(velTicks);
            shooter.flyRight.setVelocity(0.0);
        }
        else if (selectedRunType == 2)
        {
            shooter.flyLeft.setVelocity(0.0);
            shooter.flyRight.setVelocity(velTicks);
        }
        else if (selectedRunType == 3)
        {
            shooter.flyLeft.setVelocity(velTicks);
            shooter.flyRight.setVelocity(velTicks);
        }
        else
        {
            shooter.flyLeft.setVelocity(0.0);
            shooter.flyRight.setVelocity(0.0);
        }
    }

    private void recordSample(ShotSolution sol)
    {
        if (sol == null) return;

        // Use MEASURED RPMs (stronger fit than target setpoint)
        double leftRPM = velToRPM(shooter.flyLeft.getVelocity());
        double rightRPM = velToRPM(shooter.flyRight.getVelocity());

        // Average whichever wheels are active
        double rpmMeasured;
        if (selectedRunType == 1)      rpmMeasured = leftRPM;
        else if (selectedRunType == 2) rpmMeasured = rightRPM;
        else if (selectedRunType == 3) rpmMeasured = 0.5 * (leftRPM + rightRPM);
        else                           rpmMeasured = 0.0;

        if (!(rpmMeasured > 0.0)) return;

        rpmSamples.add(rpmMeasured);
        vSamples.add(sol.v);

        fitLine();
    }

    private void fitLine()
    {
        int n = rpmSamples.size();
        if (n < 2)
        {
            fitK = fitB = fitR2 = Double.NaN;
            return;
        }

        double sumR = 0, sumV = 0, sumRR = 0, sumRV = 0;
        for (int i = 0; i < n; i++)
        {
            double R = rpmSamples.get(i);
            double V = vSamples.get(i);
            sumR += R; sumV += V; sumRR += R*R; sumRV += R*V;
        }

        double meanR = sumR / n, meanV = sumV / n;
        double varR = sumRR - n*meanR*meanR;
        if (Math.abs(varR) < 1e-9)
        {
            fitK = fitB = fitR2 = Double.NaN;
            return;
        }

        fitK = (sumRV - n*meanR*meanV) / varR;
        fitB = meanV - fitK * meanR;

        // R^2
        double ssTot = 0, ssRes = 0;
        for (int i = 0; i < n; i++)
        {
            double R = rpmSamples.get(i);
            double V = vSamples.get(i);
            double Vhat = fitK * R + fitB;
            ssTot += (V - meanV)*(V - meanV);
            ssRes += (V - Vhat)*(V - Vhat);
        }
        fitR2 = (ssTot <= 1e-12) ? 1.0 : Math.max(0.0, 1.0 - ssRes/ssTot);
    }

    private void clearSamples()
    {
        rpmSamples.clear();
        vSamples.clear();
        fitK = fitB = fitR2 = Double.NaN;
    }

    private void updateTelem(ShotSolution sol)
    {
        double leftRPM = velToRPM(shooter.flyLeft.getVelocity());
        double rightRPM = velToRPM(shooter.flyRight.getVelocity());

        joinedTelemetry.addLine("=== ShotAlgorithm Calibration ===");
        joinedTelemetry.addData("Battery (V)", battery != null ? battery.getVoltage() : Double.NaN);
        joinedTelemetry.addData("Selected", selectedRunType);

        joinedTelemetry.addLine("Geometry / Bounds");
        joinedTelemetry.addData("x (m)", xMeters);
        joinedTelemetry.addData("y (m)", yMeters);
        joinedTelemetry.addData("θmin/θmax (deg)", "%.1f / %.1f", thetaMinDeg, thetaMaxDeg);
        joinedTelemetry.addData("steps", steps);

        joinedTelemetry.addLine("Live RPM");
        joinedTelemetry.addData("Left", leftRPM);
        joinedTelemetry.addData("Right", rightRPM);
        joinedTelemetry.addData("Target RPM", targetRPM);

        joinedTelemetry.addLine("Solver Result (current pose)");
        if (sol != null)
        {
            joinedTelemetry.addData("θ* (deg)", Math.toDegrees(sol.theta));
            joinedTelemetry.addData("v* (m/s)", sol.v);
        }
        else
        {
            joinedTelemetry.addLine("No feasible solution under current bounds.");
        }

        joinedTelemetry.addLine("Fit v = k*RPM + b");
        joinedTelemetry.addData("samples", rpmSamples.size());
        joinedTelemetry.addData("k (m/s per RPM)", fitK);
        joinedTelemetry.addData("b (m/s)", fitB);
        joinedTelemetry.addData("R^2", fitR2);

        joinedTelemetry.addLine("Controls");
        joinedTelemetry.addLine("X=Left  B=Right  Y=Both  A=Stop/Record   LB/RB=RPM ±" + rpmStep);
        joinedTelemetry.addLine("DPad L/R: x ±0.05m  DPad U/D: y ±0.02m  LSB/RSB: θmin-/θmax+");
        joinedTelemetry.addLine("Back=Clear samples   Start=Save JSON");

        joinedTelemetry.update();
    }

    private void saveCalibrationJson()
    {
        try
        {
            String path = "/sdcard/FIRST/calibration";
            java.io.File dir = new java.io.File(path);
            if (!dir.exists()) dir.mkdirs();
            java.io.File file = new java.io.File(dir, "shot_alg_calibration.json");

            StringBuilder sb = new StringBuilder();
            sb.append("{\n");
            sb.append(String.format("  \"k_mpsPerRPM\": %.9f,\n", fitK));
            sb.append(String.format("  \"b_mps\": %.9f,\n", fitB));
            sb.append(String.format("  \"R2\": %.6f,\n", fitR2));
            sb.append("  \"samples\": [\n");
            for (int i = 0; i < rpmSamples.size(); i++)
            {
                sb.append(String.format("    {\"rpm\": %.3f, \"v_mps\": %.6f}%s\n",
                        rpmSamples.get(i), vSamples.get(i), (i + 1 < rpmSamples.size() ? "," : "")));
            }
            sb.append("  ]\n");
            sb.append("}\n");

            try (java.io.FileWriter w = new java.io.FileWriter(file, false))
            {
                w.write(sb.toString());
            }
        }
        catch (Exception ignored) { }
    }

    // ===== Helpers =====

    private boolean pressed(boolean now, boolean before)
    {
        return now && !before;
    }

    private double velToRPM(double ticksPerSec)
    {
        double rps = ticksPerSec / ticksPerRev;
        return rps * 60.0;
    }

    private double RPMToVel(double rpm)
    {
        double tpm = rpm * ticksPerRev;
        return tpm / 60.0;
    }

    private static double clamp(double v, double lo, double hi)
    {
        return Math.max(lo, Math.min(hi, v));
    }
}
