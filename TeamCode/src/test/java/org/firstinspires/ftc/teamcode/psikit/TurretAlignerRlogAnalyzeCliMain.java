package org.firstinspires.ftc.teamcode.psikit;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class TurretAlignerRlogAnalyzeCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String HIGH_CURRENT_A_PROPERTY = "highCurrentA";

    private TurretAlignerRlogAnalyzeCliMain() {
    }

    private static final class Bucket {
        long count;
        long high2;
        long high4;
        long high6;
        double sum;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;

        void add(double amps) {
            if (Double.isNaN(amps) || Double.isInfinite(amps)) return;
            count++;
            sum += amps;
            min = Math.min(min, amps);
            max = Math.max(max, amps);
            if (amps >= 2.0) high2++;
            if (amps >= 4.0) high4++;
            if (amps >= 6.0) high6++;
        }

        double avg() {
            return count > 0 ? (sum / count) : Double.NaN;
        }
    }

    private static final class SpikeRow {
        final double ts;
        final double amps;
        final int mode;
        final boolean running;
        final boolean leftEnabled;
        final boolean rightEnabled;
        final double target;
        final double leftCmd;
        final double rightCmd;
        final double encoder;

        SpikeRow(double ts,
                 double amps,
                 int mode,
                 boolean running,
                 boolean leftEnabled,
                 boolean rightEnabled,
                 double target,
                 double leftCmd,
                 double rightCmd,
                 double encoder) {
            this.ts = ts;
            this.amps = amps;
            this.mode = mode;
            this.running = running;
            this.leftEnabled = leftEnabled;
            this.rightEnabled = rightEnabled;
            this.target = target;
            this.leftCmd = leftCmd;
            this.rightCmd = rightCmd;
            this.encoder = encoder;
        }
    }

    public static void main(String[] args) throws Exception {
        Path logPath = resolveLogPath(args);
        double highCurrentA = resolveDouble(HIGH_CURRENT_A_PROPERTY, 4.0);

        System.out.println("[TurretAlignerAnalysis] log=" + logPath.toAbsolutePath());
        System.out.println("[TurretAlignerAnalysis] highCurrentA=" + highCurrentA);

        Bucket overall = new Bucket();
        Map<Integer, Bucket> byMode = new HashMap<>();
        Map<String, Bucket> byEnableMask = new HashMap<>();
        Map<String, Bucket> byModeEnable = new HashMap<>();
        Map<Integer, Bucket> byOffsetMilliVerifyBoth = new HashMap<>();
        List<SpikeRow> spikes = new ArrayList<>();

        long totalEntries = 0;
        long entriesWithCurrent = 0;

        RLOGReplay replay = new RLOGReplay(logPath.toString());
        replay.start();
        try {
            while (true) {
                LogTable entry = replay.getEntry();
                if (entry == null) break;
                totalEntries++;

                double ts = entry.getTimestamp();
                double amps = firstFinite(
                        entry.get("RealOutputs/TurretAligner/FloodgateAmps", Double.NaN),
                        entry.get("RealOutputs/FloodgateAmps", Double.NaN)
                );
                if (Double.isNaN(amps)) continue;
                entriesWithCurrent++;

                int mode = (int) Math.round(entry.get("RealOutputs/TurretAligner/ModeSelected", Double.NaN));
                boolean running = entry.get("RealOutputs/TurretAligner/ModeRunning", 0.0) > 0.5;
                boolean leftEnabled = entry.get("RealOutputs/TurretAligner/LeftEnabled", 0.0) > 0.5;
                boolean rightEnabled = entry.get("RealOutputs/TurretAligner/RightEnabled", 0.0) > 0.5;
                double target = entry.get("RealOutputs/TurretAligner/TargetPos", Double.NaN);
                double leftCmd = entry.get("RealOutputs/TurretAligner/LeftCmd", Double.NaN);
                double rightCmd = entry.get("RealOutputs/TurretAligner/RightCmd", Double.NaN);
                double rightOffset = entry.get("RealOutputs/TurretAligner/RightOffset", Double.NaN);
                double encoder = entry.get("RealOutputs/TurretAligner/EncoderDeg", Double.NaN);

                overall.add(amps);
                byMode.computeIfAbsent(mode, k -> new Bucket()).add(amps);

                String enableMask = maskLabel(leftEnabled, rightEnabled);
                byEnableMask.computeIfAbsent(enableMask, k -> new Bucket()).add(amps);

                String modeEnable = modeLabel(mode) + "/" + enableMask;
                byModeEnable.computeIfAbsent(modeEnable, k -> new Bucket()).add(amps);

                if (mode == 1 && leftEnabled && rightEnabled && !Double.isNaN(rightOffset) && !Double.isInfinite(rightOffset)) {
                    int offsetMilli = (int) Math.round(rightOffset * 1000.0);
                    byOffsetMilliVerifyBoth.computeIfAbsent(offsetMilli, k -> new Bucket()).add(amps);
                }

                if (amps >= highCurrentA) {
                    spikes.add(new SpikeRow(ts, amps, mode, running, leftEnabled, rightEnabled, target, leftCmd, rightCmd, encoder));
                }
            }
        } finally {
            replay.end();
        }

        System.out.printf(Locale.US, "[TurretAlignerAnalysis] entries=%d withCurrent=%d%n", totalEntries, entriesWithCurrent);
        printBucket("overall", overall);

        System.out.println();
        System.out.println("[TurretAlignerAnalysis] byMode");
        byMode.entrySet().stream()
                .sorted(Map.Entry.comparingByKey())
                .forEach(e -> printBucket("  " + modeLabel(e.getKey()), e.getValue()));

        System.out.println();
        System.out.println("[TurretAlignerAnalysis] byEnableMask");
        byEnableMask.entrySet().stream()
                .sorted(Map.Entry.comparingByKey())
                .forEach(e -> printBucket("  " + e.getKey(), e.getValue()));

        System.out.println();
        System.out.println("[TurretAlignerAnalysis] byModeEnable");
        byModeEnable.entrySet().stream()
                .sorted(Map.Entry.comparingByKey())
                .forEach(e -> printBucket("  " + e.getKey(), e.getValue()));

        System.out.println();
        System.out.println("[TurretAlignerAnalysis] verifyBothByRightOffset");
        byOffsetMilliVerifyBoth.entrySet().stream()
            .sorted(Map.Entry.comparingByKey())
            .forEach(e -> {
                double offset = e.getKey() / 1000.0;
                printBucket(String.format(Locale.US, "  rightOffset=%.3f", offset), e.getValue());
            });

        System.out.println();
        System.out.printf(Locale.US, "[TurretAlignerAnalysis] spikes>=%.2fA count=%d%n", highCurrentA, spikes.size());
        spikes.sort(Comparator.comparingDouble((SpikeRow r) -> r.amps).reversed());
        int limit = Math.min(25, spikes.size());
        for (int i = 0; i < limit; i++) {
            SpikeRow row = spikes.get(i);
            System.out.printf(Locale.US,
                    "  #%02d ts=%.3f A=%.3f mode=%s running=%s en=%s target=%.4f L=%.4f R=%.4f enc=%.2f%n",
                    i + 1,
                    row.ts,
                    row.amps,
                    modeLabel(row.mode),
                    row.running,
                    maskLabel(row.leftEnabled, row.rightEnabled),
                    row.target,
                    row.leftCmd,
                    row.rightCmd,
                    row.encoder);
        }
    }

    private static void printBucket(String name, Bucket b) {
        if (b == null || b.count == 0) {
            System.out.println(name + ": count=0");
            return;
        }
        System.out.printf(Locale.US,
                "%s: count=%d avg=%.3f min=%.3f max=%.3f high2=%d high4=%d high6=%d%n",
                name,
                b.count,
                b.avg(),
                b.min,
                b.max,
                b.high2,
                b.high4,
                b.high6);
    }

    private static String modeLabel(int mode) {
        if (mode == 0) return "ALIGN";
        if (mode == 1) return "VERIFY";
        if (mode == 2) return "CHAR";
        return "UNKNOWN(" + mode + ")";
    }

    private static String maskLabel(boolean leftEnabled, boolean rightEnabled) {
        if (leftEnabled && rightEnabled) return "L1R1";
        if (leftEnabled) return "L1R0";
        if (rightEnabled) return "L0R1";
        return "L0R0";
    }

    private static double firstFinite(double a, double b) {
        if (!Double.isNaN(a) && !Double.isInfinite(a)) return a;
        if (!Double.isNaN(b) && !Double.isInfinite(b)) return b;
        return Double.NaN;
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
