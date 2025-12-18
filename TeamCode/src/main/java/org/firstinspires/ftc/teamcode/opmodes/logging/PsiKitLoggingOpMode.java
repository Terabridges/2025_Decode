package org.firstinspires.ftc.teamcode.opmodes.logging;

import org.firstinspires.ftc.teamcode.opmodes.teleop.PsiKitLoggingLinearOpMode;
import org.psilynx.psikit.core.Logger;

/**
 * Base class for iterative-style OpModes (init/init_loop/start/loop/stop) that want PsiKit logging,
 * while still allowing robot hardware to be constructed using the original FTC {@code hardwareMap}
 * and {@code gamepad1/2}.
 */
public abstract class PsiKitLoggingOpMode extends PsiKitLoggingLinearOpMode {

    /** Override if you need a different port. Keep in sync with AdvantageScope "rlogPort". */
    protected int getRlogPort() {
        return 5802;
    }

    @Override
    public final void runOpMode() {
        // Capture raw FTC objects before PsiKit wraps anything.
        captureRawFtcReferences();

        // Allow subclasses to construct robot hardware using raw FTC objects.
        psiKit_init();

        // Now enable PsiKit wrapping + priming + RLOG receivers.
        startPsiKitLogging(getRlogPort());

        try {
            while (opModeInInit() && !isStopRequested()) {
                Logger.periodicBeforeUser();
                logPsiKitInputsOncePerLoop();
                psiKit_init_loop();
                Logger.periodicAfterUser(0.0, 0.0);
            }

            if (isStopRequested()) {
                return;
            }

            psiKit_start();

            while (opModeIsActive() && !isStopRequested()) {
                Logger.periodicBeforeUser();
                logPsiKitInputsOncePerLoop();
                psiKit_loop();
                Logger.periodicAfterUser(0.0, 0.0);
            }

            psiKit_stop();
        } finally {
            endPsiKitLogging();
        }
    }

    protected abstract void psiKit_init();

    protected abstract void psiKit_init_loop();

    protected abstract void psiKit_start();

    protected abstract void psiKit_loop();

    protected abstract void psiKit_stop();
}
