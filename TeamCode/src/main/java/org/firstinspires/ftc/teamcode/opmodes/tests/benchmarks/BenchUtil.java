package org.firstinspires.ftc.teamcode.opmodes.tests.benchmarks;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

final class BenchUtil {

    private BenchUtil() {
    }

    static final class RollingStats {
        private final double[] samples;
        private int count = 0;
        private int writeIdx = 0;

        RollingStats(int windowSize) {
            this.samples = new double[Math.max(1, windowSize)];
        }

        void add(double value) {
            samples[writeIdx] = value;
            writeIdx = (writeIdx + 1) % samples.length;
            if (count < samples.length) {
                count++;
            }
        }

        int size() {
            return count;
        }

        double mean() {
            if (count <= 0) return 0.0;
            double sum = 0.0;
            for (int i = 0; i < count; i++) {
                sum += samples[i];
            }
            return sum / count;
        }

        double min() {
            if (count <= 0) return 0.0;
            double m = Double.POSITIVE_INFINITY;
            for (int i = 0; i < count; i++) {
                m = Math.min(m, samples[i]);
            }
            return m;
        }

        double max() {
            if (count <= 0) return 0.0;
            double m = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < count; i++) {
                m = Math.max(m, samples[i]);
            }
            return m;
        }

        double percentile(double p) {
            if (count <= 0) return 0.0;
            double[] copy = Arrays.copyOf(samples, count);
            Arrays.sort(copy);
            if (p <= 0.0) return copy[0];
            if (p >= 1.0) return copy[count - 1];
            int idx = (int) Math.round((count - 1) * p);
            idx = Math.max(0, Math.min(count - 1, idx));
            return copy[idx];
        }
    }

    static final class TelemetryThrottle {
        private final ElapsedTime timer = new ElapsedTime();
        private final double periodSec;

        TelemetryThrottle(double periodSec) {
            this.periodSec = Math.max(0.0, periodSec);
        }

        boolean shouldUpdate() {
            if (periodSec <= 0.0) return true;
            if (timer.seconds() >= periodSec) {
                timer.reset();
                return true;
            }
            return false;
        }
    }

    static long nowNs() {
        return System.nanoTime();
    }

    static double nsToMs(long ns) {
        return ns / 1_000_000.0;
    }

    static double nsToUs(long ns) {
        return ns / 1_000.0;
    }
}
