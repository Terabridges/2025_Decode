package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

/** Immutable configuration for a concrete autonomous variation class. */
public class AutoSpec {
    public final Range range;
    public final boolean releaseAfterClosePickup;
    public final boolean backRowLoopEnabled;
    public final boolean closeLoopEnabled;
    public final int backRowLoopCycles;
    public final boolean shootPreload;
    public final int[] rowSequence;

    public AutoSpec(
            Range range,
            boolean releaseAfterClosePickup,
            boolean backRowLoopEnabled,
            boolean shootPreload,
            int... rowSequence
    ) {
        this(range, releaseAfterClosePickup, backRowLoopEnabled, false, 0, shootPreload, rowSequence);
    }

    private AutoSpec(
            Range range,
            boolean releaseAfterClosePickup,
            boolean backRowLoopEnabled,
            boolean closeLoopEnabled,
            int backRowLoopCycles,
            boolean shootPreload,
            int... rowSequence
    ) {
        this.range = range;
        this.releaseAfterClosePickup = releaseAfterClosePickup;
        this.backRowLoopEnabled = backRowLoopEnabled;
        this.closeLoopEnabled = closeLoopEnabled;
        this.backRowLoopCycles = Math.max(0, backRowLoopCycles);
        this.shootPreload = shootPreload;
        this.rowSequence = Arrays.copyOf(rowSequence, rowSequence.length);
    }

    public static AutoSpec withBackRowLoopCycles(
            Range range,
            boolean releaseAfterClosePickup,
            boolean shootPreload,
            int backRowLoopCycles,
            int... rowSequence
    ) {
        return new AutoSpec(range, releaseAfterClosePickup, true, false, backRowLoopCycles, shootPreload, rowSequence);
    }

    public static AutoSpec withCloseLoopCycles(
            Range range,
            boolean releaseAfterClosePickup,
            boolean shootPreload,
            int closeLoopCycles,
            int... rowSequence
    ) {
        return new AutoSpec(range, releaseAfterClosePickup, true, true, closeLoopCycles, shootPreload, rowSequence);
    }
}
