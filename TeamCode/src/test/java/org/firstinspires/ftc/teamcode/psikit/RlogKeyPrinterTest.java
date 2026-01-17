package org.firstinspires.ftc.teamcode.psikit;

import org.junit.Assume;
import org.junit.Test;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LogTable.LogValue;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static org.junit.Assert.assertTrue;

/**
 * Opt-in helper test to decode an RLOG and print specific keys (including strings).
 *
 * Usage:
 *   ./gradlew :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.psikit.RlogKeyPrinterTest \
 *     -DpsikitRlogDump=true
 *
 * Optional:
 *   -DpsikitRlogPath=C:\\path\\to\\file.rlog
 *   -DpsikitRlogKeys=ReplayOutputs/LoggedRobot/ReplayInitExceptionType,ReplayOutputs/LoggedRobot/ReplayInitExceptionMessage,ReplayOutputs/LoggedRobot/ReplayInitExceptionStack
 *   -DpsikitReplayOutputDir=C:\\code\\TeraBridges\\2025_Decode\\build\\psikitReplayOut
 */
public class RlogKeyPrinterTest {

    private static final String ENABLE_PROP = "psikitRlogDump";
    private static final String ENABLE_ENV = "PSIKIT_RLOG_DUMP";
    private static final String RLOG_PATH_PROP = "psikitRlogPath";
    private static final String RLOG_PATH_ENV = "PSIKIT_RLOG_PATH";
    private static final String RLOG_KEYS_PROP = "psikitRlogKeys";
    private static final String RLOG_KEYS_ENV = "PSIKIT_RLOG_KEYS";
    private static final String RLOG_NEEDLES_PROP = "psikitRlogNeedles";
    private static final String RLOG_NEEDLES_ENV = "PSIKIT_RLOG_NEEDLES";
    private static final String REPLAY_OUT_DIR_PROP = "psikitReplayOutputDir";
    private static final String REPLAY_OUT_DIR_ENV = "PSIKIT_REPLAY_OUTPUT_DIR";

    @Test
    public void dumpKeysFromLatestReplayOutputLog() throws Exception {
        boolean enabled = Boolean.parseBoolean(System.getProperty(ENABLE_PROP, "false"));
        if (!enabled) {
            enabled = Boolean.parseBoolean(System.getenv(ENABLE_ENV));
        }
        Assume.assumeTrue(
            "Skipping: set -D" + ENABLE_PROP + "=true or env " + ENABLE_ENV + "=true to enable RLOG string dump",
                enabled
        );

        Path rlogPath = resolveRlogPath();
        assertTrue("RLOG file does not exist: " + rlogPath, Files.exists(rlogPath));

        List<String> keys = resolveKeys();

        List<String> needles = resolveNeedles(List.of("ReplayInitException", "ReplayOnly", "UserSectionMS", "Console"));

        ScanResult scan = scanLog(rlogPath, keys, needles);
        LogTable last = scan.last;

        StringBuilder out = new StringBuilder();
        out.append("=== RLOG KEY DUMP ===\n");
        out.append("Props/Env (raw):\n");
        out.append("  ").append(ENABLE_PROP).append("=").append(System.getProperty(ENABLE_PROP)).append("\n");
        out.append("  ").append(ENABLE_ENV).append("=").append(System.getenv(ENABLE_ENV)).append("\n");
        out.append("  ").append(RLOG_PATH_PROP).append("=").append(System.getProperty(RLOG_PATH_PROP)).append("\n");
        out.append("  ").append(RLOG_PATH_ENV).append("=").append(System.getenv(RLOG_PATH_ENV)).append("\n");
        out.append("  ").append(RLOG_KEYS_PROP).append("=").append(System.getProperty(RLOG_KEYS_PROP)).append("\n");
        out.append("  ").append(RLOG_KEYS_ENV).append("=").append(System.getenv(RLOG_KEYS_ENV)).append("\n");
        out.append("  ").append(RLOG_NEEDLES_PROP).append("=").append(System.getProperty(RLOG_NEEDLES_PROP)).append("\n");
        out.append("  ").append(RLOG_NEEDLES_ENV).append("=").append(System.getenv(RLOG_NEEDLES_ENV)).append("\n");
        out.append("  ").append(REPLAY_OUT_DIR_PROP).append("=").append(System.getProperty(REPLAY_OUT_DIR_PROP)).append("\n");
        out.append("  ").append(REPLAY_OUT_DIR_ENV).append("=").append(System.getenv(REPLAY_OUT_DIR_ENV)).append("\n");
        out.append("File: ").append(rlogPath).append("\n");
        out.append("Keys: ").append(keys).append("\n");
        out.append("Needles: ").append(needles).append("\n");
        out.append("Decoded cycles: ").append(scan.cycles).append("\n");

        // Diagnostics by substring: show both final-table matches and ever-seen matches.
        appendMatchingKeys(out, last, "ReplayInitException", 80);
        appendEverMatchingKeys(out, scan.everMatchingKeysByNeedle, "ReplayInitException", 80);

        appendMatchingKeys(out, last, "ReplayOnly", 80);
        appendEverMatchingKeys(out, scan.everMatchingKeysByNeedle, "ReplayOnly", 80);

        appendMatchingKeys(out, last, "UserSectionMS", 200);
        appendEverMatchingKeys(out, scan.everMatchingKeysByNeedle, "UserSectionMS", 200);

        appendMatchingKeys(out, last, "Console", 200);
        appendEverMatchingKeys(out, scan.everMatchingKeysByNeedle, "Console", 200);

        // Print any additional user-provided needles (helps debug one-off substrings without
        // editing this file each time).
        Set<String> defaultNeedles = new HashSet<>(List.of("ReplayInitException", "ReplayOnly", "UserSectionMS", "Console"));
        for (String needle : needles) {
            if (defaultNeedles.contains(needle)) continue;
            appendMatchingKeys(out, last, needle, 200);
            appendEverMatchingKeys(out, scan.everMatchingKeysByNeedle, needle, 200);
        }

        out.append("\nRequested key values (final table):\n");
        for (String key : keys) {
            LogValue v = last.get(key);
            if (v == null) {
                out.append("  ").append(key).append(" = <missing>\n");
                continue;
            }

            String rendered;
            try {
                rendered = v.toString();
            } catch (Throwable t) {
                rendered = "<toString failed: " + t.getClass().getName() + ": " + t.getMessage() + ">";
            }

            // Keep console output manageable.
            rendered = truncate(rendered, 12_000);
            out.append("  ").append(key)
                    .append(" (type=").append(v.type).append(") =\n")
                    .append(rendered).append("\n");

                if (v.type == LogTable.LoggableType.String) {
                String s = v.getString("");
                out.append("    [string length=").append(s.length())
                    .append(", cr=").append(countChar(s, '\r'))
                    .append(", lf=").append(countChar(s, '\n'))
                    .append("]\n");
                out.append("    [escaped preview] ")
                    .append(truncate(escapeForOneLine(s), 2000))
                    .append("\n");
                }
        }

        out.append("\nRequested key values (last seen in any cycle):\n");
        for (String key : keys) {
            LastSeenValue lv = scan.lastSeenByKey.get(key);
            if (lv == null) {
                out.append("  ").append(key).append(" = <never seen>\n");
                continue;
            }
            out.append("  ").append(key)
                    .append(" (cycle=").append(lv.cycle).append(", type=").append(lv.type).append(") =\n")
                    .append(truncate(lv.rendered, 12_000)).append("\n");

            if (lv.type == LogTable.LoggableType.String) {
                out.append("    [string length=").append(lv.rendered.length())
                        .append(", cr=").append(countChar(lv.rendered, '\r'))
                        .append(", lf=").append(countChar(lv.rendered, '\n'))
                        .append("]\n");
                out.append("    [escaped preview] ")
                        .append(truncate(escapeForOneLine(lv.rendered), 2000))
                        .append("\n");
            }
        }

        out.append("=== END RLOG KEY DUMP ===\n");

        // Gradle often hides test stdout; always write a dump file near replay outputs.
        Path dumpPath = resolveDumpPath(rlogPath);
        Files.createDirectories(dumpPath.getParent());
        Files.write(
            dumpPath,
            out.toString().getBytes(StandardCharsets.UTF_8),
            StandardOpenOption.CREATE,
            StandardOpenOption.TRUNCATE_EXISTING,
            StandardOpenOption.WRITE
        );

        // Still print (useful when test logging is enabled).
        System.out.println(out);
        System.out.println("Wrote RLOG key dump to: " + dumpPath);
    }

    private static List<String> resolveKeys() {
        String prop = System.getProperty(RLOG_KEYS_PROP);
        if (prop != null && !prop.trim().isEmpty()) {
            String[] parts = prop.split(",");
            List<String> out = new ArrayList<>();
            for (String p : parts) {
                String k = p.trim();
                if (!k.isEmpty()) out.add(k);
            }
            if (!out.isEmpty()) return out;
        }

        String env = System.getenv(RLOG_KEYS_ENV);
        if (env != null && !env.trim().isEmpty()) {
            String[] parts = env.split(",");
            List<String> out = new ArrayList<>();
            for (String p : parts) {
                String k = p.trim();
                if (!k.isEmpty()) out.add(k);
            }
            if (!out.isEmpty()) return out;
        }

        // Default to the fields we record in MainTeleopPsikit's replay init failure path.
        return List.of(
                "ReplayOutputs/LoggedRobot/ReplayInitExceptionType",
                "ReplayOutputs/LoggedRobot/ReplayInitExceptionMessage",
                "ReplayOutputs/LoggedRobot/ReplayInitExceptionStack"
        );
    }

    private static List<String> resolveNeedles(List<String> defaults) {
        List<String> out = new ArrayList<>();
        if (defaults != null) out.addAll(defaults);

        String prop = System.getProperty(RLOG_NEEDLES_PROP);
        if (prop != null && !prop.trim().isEmpty()) {
            for (String p : prop.split(",")) {
                String needle = p.trim();
                if (!needle.isEmpty()) out.add(needle);
            }
        }

        String env = System.getenv(RLOG_NEEDLES_ENV);
        if (env != null && !env.trim().isEmpty()) {
            for (String p : env.split(",")) {
                String needle = p.trim();
                if (!needle.isEmpty()) out.add(needle);
            }
        }

        // Deduplicate while keeping stable ordering.
        List<String> deduped = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        for (String needle : out) {
            if (seen.add(needle)) deduped.add(needle);
        }
        return deduped;
    }

    private static int countChar(String s, char c) {
        int count = 0;
        for (int i = 0; i < s.length(); i++) {
            if (s.charAt(i) == c) count++;
        }
        return count;
    }

    private static String escapeForOneLine(String s) {
        // Keep everything on a single line so whitespace is visible in logs.
        return s
                .replace("\\", "\\\\")
                .replace("\r", "\\r")
                .replace("\n", "\\n")
                .replace("\t", "\\t");
    }

    private static Path resolveRlogPath() throws IOException {
        String explicit = System.getProperty(RLOG_PATH_PROP);
        if (explicit != null && !explicit.trim().isEmpty()) {
            return Paths.get(explicit.trim());
        }

        String explicitEnv = System.getenv(RLOG_PATH_ENV);
        if (explicitEnv != null && !explicitEnv.trim().isEmpty()) {
            return Paths.get(explicitEnv.trim());
        }

        Path replayOutDir = resolveReplayOutDir();
        if (replayOutDir == null) {
            throw new IllegalStateException(
                    "Could not locate replay output dir. Set -D" + RLOG_PATH_PROP + "=... or -D" + REPLAY_OUT_DIR_PROP + "=..."
            );
        }

        try (Stream<Path> stream = Files.list(replayOutDir)) {
            return stream
                    .filter(p -> {
                        String name = p.getFileName().toString().toLowerCase(Locale.ROOT);
                        return name.endsWith(".rlog") && Files.isRegularFile(p);
                    })
                    .max(Comparator.comparingLong(p -> {
                        try {
                            return Files.getLastModifiedTime(p).toMillis();
                        } catch (IOException e) {
                            return 0L;
                        }
                    }))
                    .orElseThrow(() -> new IllegalStateException(
                            "No .rlog files found in " + replayOutDir + ". Set -D" + RLOG_PATH_PROP + "=..."
                    ));
        }
    }

    private static Path resolveReplayOutDir() {
        String prop = System.getProperty(REPLAY_OUT_DIR_PROP);
        if (prop != null && !prop.trim().isEmpty()) {
            Path p = Paths.get(prop.trim());
            if (Files.isDirectory(p)) return p;
        }

        String env = System.getenv(REPLAY_OUT_DIR_ENV);
        if (env != null && !env.trim().isEmpty()) {
            Path p = Paths.get(env.trim());
            if (Files.isDirectory(p)) return p;
        }

        // Try a few common working directories (Gradle may run tests from root or module dir).
        List<Path> candidates = List.of(
                Paths.get("build", "psikitReplayOut"),
                Paths.get("..", "build", "psikitReplayOut"),
                Paths.get(System.getProperty("user.dir"), "build", "psikitReplayOut"),
                Paths.get(System.getProperty("user.dir"), "..", "build", "psikitReplayOut")
        );

        for (Path c : candidates) {
            try {
                if (Files.isDirectory(c)) return c.normalize();
            } catch (Throwable ignored) {
            }
        }
        return null;
    }

    private static Path resolveDumpPath(Path rlogPath) {
        try {
            Path parent = rlogPath.getParent();
            if (parent != null && Files.isDirectory(parent)) {
                return parent.resolve("rlog-key-dump.txt");
            }
        } catch (Throwable ignored) {
        }
        return Paths.get("build", "rlog-key-dump.txt");
    }

    private static ScanResult scanLog(Path rlogPath, List<String> requestedKeys, List<String> needles) {
        RLOGReplay replay = new RLOGReplay(rlogPath.toString());
        replay.start();

        LogTable state = new LogTable(0);
        int cycles = 0;

        Map<String, LastSeenValue> lastSeenByKey = new HashMap<>();
        Map<String, Set<String>> everMatchingKeysByNeedle = new HashMap<>();
        for (String needle : needles) {
            everMatchingKeysByNeedle.put(needle, new HashSet<>());
        }

        while (replay.updateTable(state)) {
            cycles++;

            // Track keys ever seen for each needle.
            for (String k : state.getAll(false).keySet()) {
                for (String needle : needles) {
                    if (k.contains(needle)) {
                        everMatchingKeysByNeedle.get(needle).add(k);
                    }
                }
            }

            // Track last-seen values for requested keys.
            for (String key : requestedKeys) {
                LogValue v = state.get(key);
                if (v == null) continue;
                String rendered;
                try {
                    rendered = v.toString();
                } catch (Throwable t) {
                    rendered = "<toString failed: " + t.getClass().getName() + ": " + t.getMessage() + ">";
                }
                lastSeenByKey.put(key, new LastSeenValue(cycles, v.type, rendered));
            }
        }

        replay.end();

        assertTrue("No cycles decoded from log: " + rlogPath, cycles > 0);
        return new ScanResult(state, cycles, lastSeenByKey, everMatchingKeysByNeedle);
    }

    private static String truncate(String s, int maxChars) {
        if (s == null) return "<null>";
        if (s.length() <= maxChars) return s;
        return s.substring(0, maxChars) + "\n... (truncated)";
    }

    private static void appendMatchingKeys(StringBuilder out, LogTable table, String needle, int limit) {
        List<String> matching = table.getAll(false).keySet().stream()
                .filter(k -> k.contains(needle))
                .sorted()
                .collect(Collectors.toList());

        out.append("\nKeys containing '").append(needle).append("' in final table (")
                .append(matching.size())
                .append("):\n");

        int shown = 0;
        for (String k : matching) {
            if (shown >= limit) {
                out.append("  ... (showing first ").append(limit).append(")\n");
                break;
            }
            out.append("  - ").append(k).append("\n");
            shown++;
        }
    }

    private static void appendEverMatchingKeys(
            StringBuilder out,
            Map<String, Set<String>> everMatchingKeysByNeedle,
            String needle,
            int limit
    ) {
        Set<String> set = everMatchingKeysByNeedle.get(needle);
        if (set == null) set = Set.of();
        List<String> matching = set.stream().sorted().collect(Collectors.toList());

        out.append("\nKeys containing '").append(needle).append("' anywhere in log (")
                .append(matching.size())
                .append("):\n");

        int shown = 0;
        for (String k : matching) {
            if (shown >= limit) {
                out.append("  ... (showing first ").append(limit).append(")\n");
                break;
            }
            out.append("  - ").append(k).append("\n");
            shown++;
        }
    }

    private static final class ScanResult {
        final LogTable last;
        final int cycles;
        final Map<String, LastSeenValue> lastSeenByKey;
        final Map<String, Set<String>> everMatchingKeysByNeedle;

        ScanResult(
                LogTable last,
                int cycles,
                Map<String, LastSeenValue> lastSeenByKey,
                Map<String, Set<String>> everMatchingKeysByNeedle
        ) {
            this.last = last;
            this.cycles = cycles;
            this.lastSeenByKey = lastSeenByKey;
            this.everMatchingKeysByNeedle = everMatchingKeysByNeedle;
        }
    }

    private static final class LastSeenValue {
        final int cycle;
        final LogTable.LoggableType type;
        final String rendered;

        LastSeenValue(int cycle, LogTable.LoggableType type, String rendered) {
            this.cycle = cycle;
            this.type = type;
            this.rendered = rendered;
        }
    }
}
