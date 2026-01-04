package org.firstinspires.ftc.teamcode.config.pedroPathing;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.psilynx.psikit.ftc.FtcLogTuning;

/**
 * PedroPathing Pinpoint localizer with update() throttling.
 *
 * Pedro's stock PinpointLocalizer calls pinpoint.update() every time update() is called.
 * When Follower.update() is called every loop, that can add a large fixed-cost I2C bulk read
 * each loop. This class rate-limits the expensive update() call and reuses the last pose/velocity
 * in between.
 */
public class ThrottledPinpointLocalizer extends PinpointLocalizer {

    private long lastUpdateNs = Long.MIN_VALUE;

    public ThrottledPinpointLocalizer(HardwareMap map, PinpointConstants constants) {
        super(map, constants);
    }

    public ThrottledPinpointLocalizer(HardwareMap map, PinpointConstants constants, Pose startPose) {
        super(map, constants, startPose);
    }

    private static double secondsSince(long thenNs) {
        if (thenNs == Long.MIN_VALUE) return Double.POSITIVE_INFINITY;
        return (System.nanoTime() - thenNs) / 1_000_000_000.0;
    }

    @Override
    public void update() {
        // Share the same tuning knob as PsiKit's Pinpoint logging throttle.
        double periodSec = FtcLogTuning.pinpointReadPeriodSec;
        if (periodSec <= 0.0 || secondsSince(lastUpdateNs) >= periodSec) {
            lastUpdateNs = System.nanoTime();
            super.update();
        }
        // else: intentionally skip; PinpointLocalizer retains last pose/velocity.
    }
}
