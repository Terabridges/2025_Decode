package org.firstinspires.ftc.teamcode.config.autoUtil;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;

/**
 * Centralizes autonomous route planning decisions:
 * - which absolute rows can be run from the selected start side
 * - which score-side range to use for each shot
 * - which leave range to use after the last shot
 */
public class AutoRoutePlanner {
    private static final int MAX_FIELD_ROWS = 4;

    private final Range selectedRange;

    public AutoRoutePlanner(Range selectedRange) {
        this.selectedRange = selectedRange;
    }

    public int getStartingAbsoluteRow() {
        return (selectedRange == Range.CLOSE_RANGE) ? 1 : 2;
    }

    public int getRowsToRun() {
        return getDefaultRowSequence().length;
    }

    public int getCurrentAbsoluteRow(boolean preloadComplete, int rowsCompleted) {
        if (!preloadComplete) {
            return getStartingAbsoluteRow();
        }
        int fromCounter = getStartingAbsoluteRow() + rowsCompleted;
        return clampAbsoluteRow(fromCounter);
    }

    public Range getScoreRangeForShot(boolean preloadComplete, int currentAbsoluteRow) {
        return selectedRange;
    }

    public Range getLeaveRangeForLastShot(boolean preloadComplete, int rowsCompleted) {
        return selectedRange;
    }

    public int[] getDefaultRowSequence() {
        int start = getStartingAbsoluteRow();
        int requested = getAvailableRowsFromStart();
        int available = getAvailableRowsFromStart();
        int count = Math.max(0, Math.min(requested, available));
        int[] rows = new int[count];
        for (int i = 0; i < count; i++) {
            rows[i] = clampAbsoluteRow(start + i);
        }
        return Arrays.copyOf(rows, rows.length);
    }

    private int getAvailableRowsFromStart() {
        return MAX_FIELD_ROWS - getStartingAbsoluteRow() + 1;
    }

    private int clampAbsoluteRow(int absoluteRow) {
        return Math.max(1, Math.min(absoluteRow, MAX_FIELD_ROWS));
    }
}
