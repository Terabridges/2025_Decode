package org.firstinspires.ftc.teamcode.psikit;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LogTable.LogValue;
import org.psilynx.psikit.core.LogTable.LoggableType;
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

/**
 * Simple CLI that reads a .rlog and prints summary stats for selected key prefixes.
 *
 * Usage:
 *   .\gradlew.bat :TeamCode:runRlogSummary --no-daemon --rerun-tasks -PpsikitReplayLog="C:\\path\\file.rlog"
 * Optional:
 *   -PsummaryPrefixes=Bench/,PsiKit/sessionTimes (us)/,PsiKit/logTimes (us)/
 */
public final class RlogSummaryCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String PREFIXES_PROPERTY = "summaryPrefixes";
    private static final String KEY_CONTAINS_PROPERTY = "summaryKeyContains";

    private static final String[] DEFAULT_PREFIXES = new String[] {
            "RealOutputs/Bench/",
            "RealOutputs/PsiKit/sessionTimes (us)/",
            "RealOutputs/PsiKit/logTimes (us)/",
            "RealOutputs/LoggedRobot/",
    };

    private RlogSummaryCliMain() {
    }

    public static void main(String[] args) throws Exception {
        Path logPath = resolveLogPathFromPropsOrArgs(args);
        String[] prefixes = resolvePrefixes();
        String keyContains = resolveString(KEY_CONTAINS_PROPERTY, "");

        System.out.println("[RlogSummary] log=" + logPath.toAbsolutePath());
        System.out.println("[RlogSummary] prefixes=" + String.join(",", prefixes));
        if (!keyContains.isEmpty()) {
            System.out.println("[RlogSummary] keyContains=" + keyContains);
        }

        Map<String, Series> seriesByKey = new HashMap<>();
        long entryCount = 0;
        long keyCount = 0;
        long matchedNumericCount = 0;
        List<String> firstEntryKeyDump = new ArrayList<>();

        RLOGReplay replay = new RLOGReplay(logPath.toString());
        replay.start();
        try {
            while (true) {
                LogTable entry = replay.getEntry();
                if (entry == null) {
                    break;
                }

                entryCount++;

                for (Map.Entry<String, LogValue> e : entry.getAll(false).entrySet()) {
                    String key = e.getKey();
                    keyCount++;

                    if (entryCount == 1 && firstEntryKeyDump.size() < 60) {
                        LogValue v = e.getValue();
                        String type = v != null ? String.valueOf(v.type) : "null";
                        String custom = (v != null && v.customTypeStr != null) ? (" custom=" + v.customTypeStr) : "";
                        firstEntryKeyDump.add(key + " type=" + type + custom);
                    }

                    if (!matchesAnyPrefix(key, prefixes)) {
                        continue;
                    }

                    if (!keyContains.isEmpty() && !key.contains(keyContains)) {
                        continue;
                    }

                    Double v = coerceToDouble(e.getValue());
                    if (v == null) {
                        continue;
                    }

                    matchedNumericCount++;

                    seriesByKey.computeIfAbsent(key, k -> new Series()).add(v);
                }
            }
        } finally {
            replay.end();
        }

        if (seriesByKey.isEmpty()) {
            System.out.println("[RlogSummary] No matching numeric keys found.");
            System.out.println("[RlogSummary] entries=" + entryCount + " keysScanned=" + keyCount + " matchedNumeric=" + matchedNumericCount);
            if (!firstEntryKeyDump.isEmpty()) {
                System.out.println("[RlogSummary] First entry keys (sample):");
                for (String line : firstEntryKeyDump) {
                    System.out.println("  " + line);
                }
            }
            return;
        }

        printOneGroup("ALL", seriesByKey);

        System.out.println();
        System.out.println("[RlogSummary] done");
    }

    private static void printOneGroup(String groupName, Map<String, Series> seriesByKey) {
        List<Map.Entry<String, Series>> rows = new ArrayList<>(seriesByKey.entrySet());
        rows.sort(Comparator.comparing(Map.Entry::getKey));

        System.out.println();
        System.out.println("group,key,count,avg,p50,p95,p99,min,max");
        for (Map.Entry<String, Series> row : rows) {
            String key = row.getKey();
            Series s = row.getValue();
            s.finish();
            System.out.printf(Locale.US,
                    "%s,%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                    groupName,
                    csvEscape(key),
                    s.count,
                    s.mean(),
                    s.p50,
                    s.p95,
                    s.p99,
                    s.min,
                    s.max
            );
        }
    }

    private static Path resolveLogPathFromPropsOrArgs(String[] args) {
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

        throw new IllegalArgumentException(
                "Missing log path. Pass -D" + LOG_PATH_PROPERTY + "=... or --log <path>"
        );
    }

    private static void requireExists(Path p) {
        if (!Files.exists(p)) {
            throw new IllegalArgumentException("Log file does not exist: " + p.toAbsolutePath());
        }
    }

    private static String[] resolvePrefixes() {
        String raw = System.getProperty(PREFIXES_PROPERTY);
        if (raw == null || raw.trim().isEmpty()) {
            return DEFAULT_PREFIXES;
        }
        String[] parts = raw.split(",");
        List<String> out = new ArrayList<>();
        for (String p : parts) {
            String t = p.trim();
            if (!t.isEmpty()) out.add(t);
        }
        return out.isEmpty() ? DEFAULT_PREFIXES : out.toArray(new String[0]);
    }

    private static boolean matchesAnyPrefix(String key, String[] prefixes) {
        for (String p : prefixes) {
            if (key.startsWith(p)) {
                return true;
            }
        }
        return false;
    }

    private static Double coerceToDouble(LogValue value) {
        if (value == null) return null;
        LoggableType t = value.type;
        switch (t) {
            case Double:
                return value.getDouble();
            case Float:
                return (double) value.getFloat();
            case Integer:
                return (double) value.getInteger();
            case Boolean:
                return value.getBoolean() ? 1.0 : 0.0;
            default:
                return null;
        }
    }

    private static String resolveString(String prop, String defaultValue) {
        String raw = System.getProperty(prop);
        if (raw == null) return defaultValue;
        String t = raw.trim();
        return t.isEmpty() ? defaultValue : t;
    }


    private static String csvEscape(String s) {
        if (s == null) return "";
        boolean needsQuote = s.contains(",") || s.contains("\"") || s.contains("\n") || s.contains("\r");
        if (!needsQuote) return s;
        return "\"" + s.replace("\"", "\"\"") + "\"";
    }

    private static final class Series {
        private final ArrayList<Double> values = new ArrayList<>();
        long count = 0;
        double sum = 0.0;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;

        double p50 = 0.0;
        double p95 = 0.0;
        double p99 = 0.0;

        void add(double v) {
            values.add(v);
            count++;
            sum += v;
            min = Math.min(min, v);
            max = Math.max(max, v);
        }

        double mean() {
            return count > 0 ? (sum / count) : 0.0;
        }

        void finish() {
            if (values.isEmpty()) {
                p50 = p95 = p99 = 0.0;
                min = max = 0.0;
                return;
            }
            values.sort(Double::compareTo);
            p50 = percentile(0.50);
            p95 = percentile(0.95);
            p99 = percentile(0.99);
        }

        private double percentile(double p) {
            if (values.isEmpty()) return 0.0;
            if (p <= 0.0) return values.get(0);
            if (p >= 1.0) return values.get(values.size() - 1);
            int idx = (int) Math.round((values.size() - 1) * p);
            idx = Math.max(0, Math.min(values.size() - 1, idx));
            return values.get(idx);
        }
    }
}
