package org.firstinspires.ftc.teamcode.config.subsystems;

import org.psilynx.psikit.core.Logger;

public interface Subsystem {
    void update();
    void toInit();

    default void updateWithTiming() {
        long startNs = System.nanoTime();
        update();
        long endNs = System.nanoTime();
        Logger.recordOutput(
                "SubsystemTiming/" + getSubsystemName() + "/UpdateMs",
                (endNs - startNs) / 1_000_000.0
        );
    }

    default String getSubsystemName() {
        return getClass().getSimpleName();
    }
}
