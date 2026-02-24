package org.firstinspires.ftc.teamcode.config.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopTimeTracker {
    private static final int DEFAULT_AVERAGE_WINDOW_SIZE = 25;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final double[] samples;

    private int sampleCount = 0;
    private int sampleIndex = 0;
    private double sampleSumMs = 0.0;
    private double currentLoopTimeMs = 0.0;
    private double trailingAverageMs = 0.0;

    public LoopTimeTracker() {
        this(DEFAULT_AVERAGE_WINDOW_SIZE);
    }

    public LoopTimeTracker(int averageWindowSize) {
        int safeWindow = Math.max(1, averageWindowSize);
        this.samples = new double[safeWindow];
    }

    public void reset() {
        loopTimer.reset();
        sampleCount = 0;
        sampleIndex = 0;
        sampleSumMs = 0.0;
        currentLoopTimeMs = 0.0;
        trailingAverageMs = 0.0;
    }

    public void sampleLoop() {
        currentLoopTimeMs = loopTimer.milliseconds();
        addSample(currentLoopTimeMs);
        loopTimer.reset();
    }

    public double getCurrentLoopTimeMs() {
        return currentLoopTimeMs;
    }

    public double getTrailingAverageMs() {
        return trailingAverageMs;
    }

    private void addSample(double sampleMs) {
        if (sampleCount < samples.length) {
            samples[sampleIndex] = sampleMs;
            sampleSumMs += sampleMs;
            sampleCount++;
            sampleIndex = (sampleIndex + 1) % samples.length;
        } else {
            double replaced = samples[sampleIndex];
            samples[sampleIndex] = sampleMs;
            sampleSumMs += sampleMs - replaced;
            sampleIndex = (sampleIndex + 1) % samples.length;
        }

        trailingAverageMs = sampleCount > 0 ? sampleSumMs / sampleCount : 0.0;
    }
}