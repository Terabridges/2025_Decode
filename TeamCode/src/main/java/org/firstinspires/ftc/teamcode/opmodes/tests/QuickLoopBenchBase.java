package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.psilynx.psikit.core.Logger;

import java.util.Arrays;

public abstract class QuickLoopBenchBase extends OpMode {

    private static final int SAMPLE_CAPACITY = 1200;

    private final double[] samplesMs = new double[SAMPLE_CAPACITY];
    private int sampleCount = 0;
    private int nextSampleIndex = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    private double sumMs = 0.0;
    private double maxMs = 0.0;

    @Override
    public final void init() {
        onBenchInit();
        loopTimer.reset();
        telemetryTimer.reset();
    }

    @Override
    public void start() {
        loopTimer.reset();
        telemetryTimer.reset();
    }

    @Override
    public final void loop() {
        final double loopDtMs = loopTimer.milliseconds();
        loopTimer.reset();

        addSample(loopDtMs);

        if (isPsiKitLoopLoggingEnabled()) {
            Logger.recordOutput("Bench/LoopDtMs", loopDtMs);
            Logger.recordOutput("Bench/Mode", benchName());
        }

        onBenchLoop();

        if (telemetryTimer.milliseconds() > 200.0) {
            telemetry.addData("Bench", benchName());
            telemetry.addData("Samples", sampleCount);
            telemetry.addData("LoopDt avg (ms)", sampleCount == 0 ? 0.0 : sumMs / sampleCount);
            telemetry.addData("LoopDt p95 (ms)", computeP95Ms());
            telemetry.addData("LoopDt max (ms)", maxMs);
            telemetry.update();
            telemetryTimer.reset();
        }
    }

    protected void onBenchInit() {
    }

    protected void onBenchLoop() {
    }

    protected boolean isPsiKitLoopLoggingEnabled() {
        return false;
    }

    protected abstract String benchName();

    private void addSample(double loopDtMs) {
        if (sampleCount < SAMPLE_CAPACITY) {
            samplesMs[nextSampleIndex] = loopDtMs;
            sampleCount++;
        } else {
            sumMs -= samplesMs[nextSampleIndex];
            samplesMs[nextSampleIndex] = loopDtMs;
        }

        nextSampleIndex = (nextSampleIndex + 1) % SAMPLE_CAPACITY;
        sumMs += loopDtMs;
        if (loopDtMs > maxMs) {
            maxMs = loopDtMs;
        }
    }

    private double computeP95Ms() {
        if (sampleCount == 0) {
            return 0.0;
        }

        double[] copy = Arrays.copyOf(samplesMs, sampleCount);
        Arrays.sort(copy);

        int index = (int) Math.ceil(0.95 * sampleCount) - 1;
        if (index < 0) {
            index = 0;
        }
        return copy[index];
    }
}
