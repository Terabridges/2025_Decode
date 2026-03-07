package org.firstinspires.ftc.teamcode.psikit;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class SpindexTestRlogAnalyzeCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String MIN_DEG_PROPERTY = "minDeg";
    private static final String MAX_DEG_PROPERTY = "maxDeg";
    private static final String WRAP_JUMP_DEG_PROPERTY = "wrapJumpDeg";
    private static final String WRAP_MISMATCH_DEG_PROPERTY = "wrapMismatchDeg";

    private SpindexTestRlogAnalyzeCliMain() {
    }

    public static void main(String[] args) throws Exception {
        Path logPath = resolveLogPath(args);
        double minDeg = resolveDouble(MIN_DEG_PROPERTY, 60.0);
        double maxDeg = resolveDouble(MAX_DEG_PROPERTY, 680.0);
        double wrapJumpDeg = resolveDouble(WRAP_JUMP_DEG_PROPERTY, 220.0);
        double wrapMismatchDeg = resolveDouble(WRAP_MISMATCH_DEG_PROPERTY, 220.0);

        System.out.println("[SpindexTestAnalysis] log=" + logPath.toAbsolutePath());
        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] filters: minDeg=%.3f maxDeg=%.3f wrapJumpDeg=%.3f wrapMismatchDeg=%.3f%n",
                minDeg, maxDeg, wrapJumpDeg, wrapMismatchDeg);

        long total = 0;
        long missing = 0;
        long outOfRange = 0;
        long wrapJumpRejected = 0;
        long wrapMismatchRejected = 0;

        List<Double> cmd = new ArrayList<>();
        List<Double> abs = new ArrayList<>();
        List<Double> err = new ArrayList<>();

        double prevCmd = Double.NaN;
        double prevAbs = Double.NaN;

        RLOGReplay replay = new RLOGReplay(logPath.toString());
        replay.start();
        try {
            while (true) {
                LogTable entry = replay.getEntry();
                if (entry == null) break;
                total++;

                double commanded = entry.get("RealOutputs/SpindexTest/CommandedDeg", Double.NaN);
                double absolute = entry.get("RealOutputs/SpindexTest/AbsoluteDeg", Double.NaN);

                if (!isFinite(commanded) || !isFinite(absolute)) {
                    missing++;
                    continue;
                }

                if (commanded < minDeg || commanded > maxDeg || absolute < minDeg || absolute > maxDeg) {
                    outOfRange++;
                    prevCmd = commanded;
                    prevAbs = absolute;
                    continue;
                }

                if (isFinite(prevCmd) && isFinite(prevAbs)) {
                    if (Math.abs(commanded - prevCmd) > wrapJumpDeg || Math.abs(absolute - prevAbs) > wrapJumpDeg) {
                        wrapJumpRejected++;
                        prevCmd = commanded;
                        prevAbs = absolute;
                        continue;
                    }
                }

                double e = commanded - absolute;
                if (Math.abs(e) > wrapMismatchDeg) {
                    wrapMismatchRejected++;
                    prevCmd = commanded;
                    prevAbs = absolute;
                    continue;
                }

                cmd.add(commanded);
                abs.add(absolute);
                err.add(e);
                prevCmd = commanded;
                prevAbs = absolute;
            }
        } finally {
            replay.end();
        }

        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] total=%d accepted=%d missing=%d outOfRange=%d wrapJumpRejected=%d wrapMismatchRejected=%d%n",
                total, cmd.size(), missing, outOfRange, wrapJumpRejected, wrapMismatchRejected);

        if (cmd.isEmpty()) {
            System.out.println("[SpindexTestAnalysis] no accepted samples after filtering");
            return;
        }

        Stats cmdStats = Stats.of(cmd);
        Stats absStats = Stats.of(abs);
        Stats errStats = Stats.of(err);

        double[] fit = linearFit(cmd, abs);
        double slope = fit[0];
        double intercept = fit[1];

        double recommendedBiasDelta = -errStats.p50;

        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] cmd: min=%.3f p50=%.3f p95=%.3f max=%.3f%n",
                cmdStats.min, cmdStats.p50, cmdStats.p95, cmdStats.max);
        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] abs: min=%.3f p50=%.3f p95=%.3f max=%.3f%n",
                absStats.min, absStats.p50, absStats.p95, absStats.max);
        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] err(cmd-abs): mean=%.3f p50=%.3f p95=%.3f p99=%.3f min=%.3f max=%.3f%n",
                errStats.mean, errStats.p50, errStats.p95, errStats.p99, errStats.min, errStats.max);
        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] fit(abs ~= slope*cmd + intercept): slope=%.6f intercept=%.6f%n",
                slope, intercept);
        System.out.printf(Locale.US,
                "[SpindexTestAnalysis] recommended: biasDelta=%.3f deg (newBias = oldBias + biasDelta)%n",
                recommendedBiasDelta);
    }

    private static final class Stats {
        final double min;
        final double max;
        final double mean;
        final double p50;
        final double p95;
        final double p99;

        private Stats(double min, double max, double mean, double p50, double p95, double p99) {
            this.min = min;
            this.max = max;
            this.mean = mean;
            this.p50 = p50;
            this.p95 = p95;
            this.p99 = p99;
        }

        static Stats of(List<Double> values) {
            List<Double> sorted = new ArrayList<>(values);
            Collections.sort(sorted);
            double sum = 0.0;
            for (double v : sorted) sum += v;
            return new Stats(
                    sorted.get(0),
                    sorted.get(sorted.size() - 1),
                    sum / sorted.size(),
                    percentile(sorted, 0.50),
                    percentile(sorted, 0.95),
                    percentile(sorted, 0.99)
            );
        }

        private static double percentile(List<Double> sorted, double q) {
            if (sorted.isEmpty()) return Double.NaN;
            double idx = q * (sorted.size() - 1);
            int lo = (int) Math.floor(idx);
            int hi = (int) Math.ceil(idx);
            if (lo == hi) return sorted.get(lo);
            double t = idx - lo;
            return sorted.get(lo) * (1.0 - t) + sorted.get(hi) * t;
        }
    }

    private static double[] linearFit(List<Double> x, List<Double> y) {
        int n = Math.min(x.size(), y.size());
        if (n < 2) return new double[]{Double.NaN, Double.NaN};

        double sumX = 0.0;
        double sumY = 0.0;
        for (int i = 0; i < n; i++) {
            sumX += x.get(i);
            sumY += y.get(i);
        }
        double meanX = sumX / n;
        double meanY = sumY / n;

        double sxx = 0.0;
        double sxy = 0.0;
        for (int i = 0; i < n; i++) {
            double dx = x.get(i) - meanX;
            double dy = y.get(i) - meanY;
            sxx += dx * dx;
            sxy += dx * dy;
        }

        if (Math.abs(sxx) < 1e-12) return new double[]{Double.NaN, Double.NaN};
        double slope = sxy / sxx;
        double intercept = meanY - (slope * meanX);
        return new double[]{slope, intercept};
    }

    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static Path resolveLogPath(String[] args) {
        String fromProp = System.getProperty(LOG_PATH_PROPERTY);
        if (fromProp != null && !fromProp.trim().isEmpty()) {
            Path p = Paths.get(fromProp.trim());
            requireExists(p);
            return p;
        }

        for (int i = 0; i < args.length; i++) {
            if ("--log".equals(args[i]) && i + 1 < args.length) {
                Path p = Paths.get(args[i + 1]);
                requireExists(p);
                return p;
            }
        }

        throw new IllegalArgumentException("Missing log path. Pass -D" + LOG_PATH_PROPERTY + "=... or --log <path>");
    }

    private static void requireExists(Path p) {
        if (!Files.exists(p)) {
            throw new IllegalArgumentException("Log file does not exist: " + p.toAbsolutePath());
        }
    }

    private static double resolveDouble(String property, double defaultValue) {
        String raw = System.getProperty(property);
        if (raw == null || raw.trim().isEmpty()) return defaultValue;
        try {
            return Double.parseDouble(raw.trim());
        } catch (Exception ignored) {
            return defaultValue;
        }
    }
}
