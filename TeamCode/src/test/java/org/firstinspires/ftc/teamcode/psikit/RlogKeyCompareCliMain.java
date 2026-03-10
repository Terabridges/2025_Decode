package org.firstinspires.ftc.teamcode.psikit;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LogTable.LogValue;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Locale;

/**
 * Compare two RLOG keys entry-by-entry and report equality stats.
 *
 * Usage:
 *   .\gradlew.bat :TeamCode:runRlogKeyCompare --no-daemon --rerun-tasks
 *     -PpsikitReplayLog="C:\\path\\file.rlog"
 *     -PcompareKeyA="RealOutputs/Localization/LimelightMT1/Pose2dRaw"
 *     -PcompareKeyB="HardwareMap/limelight/field/botPose2d"
 */
public final class RlogKeyCompareCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String KEY_A_PROPERTY = "compareKeyA";
    private static final String KEY_B_PROPERTY = "compareKeyB";
    private static final String IGNORE_ZERO_POSE_PROPERTY = "compareIgnoreZeroPose2dRaw";
    private static final String VALIDITY_KEY_PROPERTY = "compareValidityKey";
    private static final String REQUIRE_VALID_PROPERTY = "compareRequireValid";

    private static final String DEFAULT_KEY_A = "RealOutputs/Localization/LimelightMT1/Pose2dRaw";
    private static final String DEFAULT_KEY_B = "HardwareMap/limelight/field/botPose2d";

    private RlogKeyCompareCliMain() {}

    public static void main(String[] args) {
        Path logPath = resolveRequiredPath(LOG_PATH_PROPERTY);
        String keyA = resolveString(KEY_A_PROPERTY, DEFAULT_KEY_A);
        String keyB = resolveString(KEY_B_PROPERTY, DEFAULT_KEY_B);
        boolean ignoreZeroPoseRaw = resolveBoolean(IGNORE_ZERO_POSE_PROPERTY, false);
        String validityKey = resolveString(VALIDITY_KEY_PROPERTY, "");
        boolean requireValid = resolveBoolean(REQUIRE_VALID_PROPERTY, false);

        System.out.println("[RlogKeyCompare] log=" + logPath.toAbsolutePath());
        System.out.println("[RlogKeyCompare] keyA=" + keyA);
        System.out.println("[RlogKeyCompare] keyB=" + keyB);
        System.out.println("[RlogKeyCompare] ignoreZeroPose2dRaw=" + ignoreZeroPoseRaw);
        System.out.println("[RlogKeyCompare] validityKey=" + (validityKey.isEmpty() ? "<none>" : validityKey));
        System.out.println("[RlogKeyCompare] requireValid=" + requireValid);

        long entries = 0;
        long aPresent = 0;
        long bPresent = 0;
        long bothPresent = 0;
        long filteredOutByZero = 0;
        long filteredOutByValidity = 0;
        long equalCount = 0;
        long differentCount = 0;

        String aType = null;
        String bType = null;

        long firstDifferentEntry = -1;
        double firstDifferentTimestamp = Double.NaN;
        String firstDifferentReason = null;
        String firstDifferentAPreview = null;
        String firstDifferentBPreview = null;

        RLOGReplay replay = new RLOGReplay(logPath.toString());
        replay.start();
        try {
            while (true) {
                LogTable entry = replay.getEntry();
                if (entry == null) break;
                entries++;

                LogValue va = entry.get(keyA);
                LogValue vb = entry.get(keyB);

                if (va != null) {
                    aPresent++;
                    if (aType == null) {
                        aType = describeType(va);
                    }
                }
                if (vb != null) {
                    bPresent++;
                    if (bType == null) {
                        bType = describeType(vb);
                    }
                }

                if (va == null || vb == null) {
                    continue;
                }

                if (requireValid && !validityKey.isEmpty()) {
                    LogValue vv = entry.get(validityKey);
                    boolean isValid = vv != null && isTruthy(vv);
                    if (!isValid) {
                        filteredOutByValidity++;
                        continue;
                    }
                }

                if (ignoreZeroPoseRaw && (isZeroPose2dRaw(va) || isZeroPose2dRaw(vb))) {
                    filteredOutByZero++;
                    continue;
                }

                bothPresent++;
                CompareResult cmp = compare(va, vb);
                if (cmp.equal) {
                    equalCount++;
                } else {
                    differentCount++;
                    if (firstDifferentEntry < 0) {
                        firstDifferentEntry = entries;
                        firstDifferentTimestamp = entry.getTimestamp();
                        firstDifferentReason = cmp.reason;
                        firstDifferentAPreview = cmp.aPreview;
                        firstDifferentBPreview = cmp.bPreview;
                    }
                }
            }
        } finally {
            replay.end();
        }

        System.out.println("[RlogKeyCompare] entries=" + entries);
        System.out.println("[RlogKeyCompare] keyA.present=" + aPresent + " type=" + nullSafe(aType));
        System.out.println("[RlogKeyCompare] keyB.present=" + bPresent + " type=" + nullSafe(bType));
        System.out.println("[RlogKeyCompare] bothPresent=" + bothPresent);
        System.out.println("[RlogKeyCompare] filteredOutByZero=" + filteredOutByZero);
        System.out.println("[RlogKeyCompare] filteredOutByValidity=" + filteredOutByValidity);
        System.out.println("[RlogKeyCompare] equal=" + equalCount + " different=" + differentCount);

        if (firstDifferentEntry >= 0) {
            System.out.println("[RlogKeyCompare] firstDifferent.entry=" + firstDifferentEntry);
            System.out.println("[RlogKeyCompare] firstDifferent.timestampSec=" + firstDifferentTimestamp);
            System.out.println("[RlogKeyCompare] firstDifferent.reason=" + firstDifferentReason);
            if (firstDifferentAPreview != null) {
                System.out.println("[RlogKeyCompare] firstDifferent.aPreview=" + firstDifferentAPreview);
            }
            if (firstDifferentBPreview != null) {
                System.out.println("[RlogKeyCompare] firstDifferent.bPreview=" + firstDifferentBPreview);
            }
        }
    }

    private static CompareResult compare(LogValue a, LogValue b) {
        if (a.type != b.type) {
            return CompareResult.notEqual("typeMismatch:" + a.type + " vs " + b.type, preview(a), preview(b));
        }

        String aCustom = a.customTypeStr;
        String bCustom = b.customTypeStr;
        if (aCustom == null ? bCustom != null : !aCustom.equals(bCustom)) {
            return CompareResult.notEqual(
                    "customTypeMismatch:" + nullSafe(aCustom) + " vs " + nullSafe(bCustom),
                    preview(a),
                    preview(b)
            );
        }

        switch (a.type) {
            case Raw:
                byte[] ar = a.getRaw();
                byte[] br = b.getRaw();
                if (Arrays.equals(ar, br)) {
                    return CompareResult.equal();
                }
                return CompareResult.notEqual(
                        "rawBytesDifferent(lenA=" + ar.length + ",lenB=" + br.length + ")",
                        previewRaw(ar),
                        previewRaw(br)
                );
            case Boolean:
                return a.getBoolean() == b.getBoolean()
                        ? CompareResult.equal()
                        : CompareResult.notEqual("booleanDifferent", preview(a), preview(b));
            case Integer:
                return a.getInteger() == b.getInteger()
                        ? CompareResult.equal()
                        : CompareResult.notEqual("integerDifferent", preview(a), preview(b));
            case Float:
                return Float.compare(a.getFloat(), b.getFloat()) == 0
                        ? CompareResult.equal()
                        : CompareResult.notEqual("floatDifferent", preview(a), preview(b));
            case Double:
                return Double.compare(a.getDouble(), b.getDouble()) == 0
                        ? CompareResult.equal()
                        : CompareResult.notEqual("doubleDifferent", preview(a), preview(b));
            case String:
                return a.getString().equals(b.getString())
                        ? CompareResult.equal()
                        : CompareResult.notEqual("stringDifferent", preview(a), preview(b));
            case BooleanArray:
                return Arrays.equals(a.getBooleanArray(), b.getBooleanArray())
                        ? CompareResult.equal()
                        : CompareResult.notEqual("booleanArrayDifferent", preview(a), preview(b));
            case IntegerArray:
                return Arrays.equals(a.getIntegerArray(), b.getIntegerArray())
                        ? CompareResult.equal()
                        : CompareResult.notEqual("integerArrayDifferent", preview(a), preview(b));
            case FloatArray:
                return Arrays.equals(a.getFloatArray(), b.getFloatArray())
                        ? CompareResult.equal()
                        : CompareResult.notEqual("floatArrayDifferent", preview(a), preview(b));
            case DoubleArray:
                return Arrays.equals(a.getDoubleArray(), b.getDoubleArray())
                        ? CompareResult.equal()
                        : CompareResult.notEqual("doubleArrayDifferent", preview(a), preview(b));
            case StringArray:
                return Arrays.equals(a.getStringArray(), b.getStringArray())
                        ? CompareResult.equal()
                        : CompareResult.notEqual("stringArrayDifferent", preview(a), preview(b));
            default:
                return CompareResult.notEqual("unsupportedType:" + a.type, preview(a), preview(b));
        }
    }

    private static String preview(LogValue value) {
        if (value == null) return "<null>";
        if (value.type == LogTable.LoggableType.Raw) {
            return previewRaw(value.getRaw());
        }
        return value.toString();
    }

    private static String previewRaw(byte[] data) {
        int max = Math.min(data.length, 24);
        StringBuilder sb = new StringBuilder();
        sb.append("len=").append(data.length).append(" hex=");
        for (int i = 0; i < max; i++) {
            if (i > 0) sb.append(' ');
            sb.append(String.format("%02X", data[i]));
        }
        if (data.length > max) {
            sb.append(" ...");
        }
        double[] maybeTriplet = decodeThreeDoublesLittleEndian(data);
        if (maybeTriplet != null) {
            sb.append(" doubles=[")
                    .append(maybeTriplet[0]).append(",")
                    .append(maybeTriplet[1]).append(",")
                    .append(maybeTriplet[2]).append("]");
        }
        return sb.toString();
    }

    private static double[] decodeThreeDoublesLittleEndian(byte[] data) {
        if (data == null || data.length != 24) {
            return null;
        }
        ByteBuffer bb = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        return new double[] { bb.getDouble(), bb.getDouble(), bb.getDouble() };
    }

    private static String describeType(LogValue value) {
        return value.type + (value.customTypeStr != null ? (" custom=" + value.customTypeStr) : "");
    }

    private static boolean isZeroPose2dRaw(LogValue value) {
        if (value == null || value.type != LogTable.LoggableType.Raw) {
            return false;
        }
        if (value.customTypeStr == null || !"struct:Pose2d".equals(value.customTypeStr)) {
            return false;
        }
        double[] d = decodeThreeDoublesLittleEndian(value.getRaw());
        if (d == null) {
            return false;
        }
        return Double.compare(d[0], 0.0) == 0
                && Double.compare(d[1], 0.0) == 0
                && Double.compare(d[2], 0.0) == 0;
    }

    private static boolean isTruthy(LogValue value) {
        if (value == null) return false;
        switch (value.type) {
            case Boolean:
                return value.getBoolean();
            case Integer:
                return value.getInteger() != 0;
            case Float:
                return Float.compare(value.getFloat(), 0f) != 0;
            case Double:
                return Double.compare(value.getDouble(), 0.0) != 0;
            case String:
                String s = value.getString();
                if (s == null) return false;
                String t = s.trim().toLowerCase(Locale.ROOT);
                return "1".equals(t) || "true".equals(t) || "yes".equals(t) || "on".equals(t);
            default:
                return false;
        }
    }

    private static String nullSafe(String input) {
        return input == null ? "<null>" : input;
    }

    private static Path resolveRequiredPath(String prop) {
        String raw = System.getProperty(prop);
        if (raw == null || raw.trim().isEmpty()) {
            throw new IllegalArgumentException("Missing -D" + prop + " system property");
        }
        Path p = Paths.get(raw.trim());
        if (!Files.exists(p)) {
            throw new IllegalArgumentException("Log file does not exist: " + p.toAbsolutePath());
        }
        return p;
    }

    private static String resolveString(String prop, String defaultValue) {
        String raw = System.getProperty(prop);
        if (raw == null) return defaultValue;
        String t = raw.trim();
        return t.isEmpty() ? defaultValue : t;
    }

    private static boolean resolveBoolean(String prop, boolean defaultValue) {
        String raw = System.getProperty(prop);
        if (raw == null) return defaultValue;
        String t = raw.trim().toLowerCase(Locale.ROOT);
        if (t.isEmpty()) return defaultValue;
        return "1".equals(t) || "true".equals(t) || "yes".equals(t) || "on".equals(t);
    }

    private static final class CompareResult {
        final boolean equal;
        final String reason;
        final String aPreview;
        final String bPreview;

        private CompareResult(boolean equal, String reason, String aPreview, String bPreview) {
            this.equal = equal;
            this.reason = reason;
            this.aPreview = aPreview;
            this.bPreview = bPreview;
        }

        static CompareResult equal() {
            return new CompareResult(true, null, null, null);
        }

        static CompareResult notEqual(String reason, String aPreview, String bPreview) {
            return new CompareResult(false, reason, aPreview, bPreview);
        }
    }
}