package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Logs goBILDA Pinpoint (FTC SDK {@link GoBildaPinpointDriver}) pose in AdvantageScope-friendly
 * schemas for 2D/3D visualizations.
 */
public final class PsiKitPinpointV2Logger {

    private final List<NamedPinpoint> cached = new ArrayList<>();
    private boolean cachedOnce;

    /**
     * Logs all configured Pinpoint devices found in the {@link HardwareMap}.
     *
     * <p>On the first call, discovers devices and caches them. Each call then:
     * <ol>
     *   <li>Calls {@link GoBildaPinpointDriver#update()}</li>
     *   <li>Reads {@link GoBildaPinpointDriver#getPosition()}</li>
     *   <li>Logs Pose2d + Pose3d representations</li>
     * </ol>
     */
    public void logAll(HardwareMap hardwareMap) {
        if (!cachedOnce) {
            cacheDevices(hardwareMap);
            cachedOnce = true;
        }

        for (int i = 0; i < cached.size(); i++) {
            NamedPinpoint device = cached.get(i);

            device.pinpoint.update();
            Pose2D pose = device.pinpoint.getPosition();

            double xMeters = pose.getX(DistanceUnit.METER);
            double yMeters = pose.getY(DistanceUnit.METER);
            double headingRad = pose.getHeading(AngleUnit.RADIANS);

            // Log Pose2d/Pose3d as struct fields so AdvantageScope can select the whole pose.
            device.poses.set(xMeters, yMeters, headingRad);
            Logger.processInputs("/Odometry/" + device.name, device.poses);

            // Convenience aliases when there's only one Pinpoint configured.
            if (cached.size() == 1) {
                robotAliases.set(xMeters, yMeters, headingRad);
                Logger.processInputs("/Odometry", robotAliases);
            }
        }
    }

    private void cacheDevices(HardwareMap hardwareMap) {
        cached.clear();

        List<GoBildaPinpointDriver> devices = hardwareMap.getAll(GoBildaPinpointDriver.class);
        for (GoBildaPinpointDriver device : devices) {
            String name = firstNameOrFallback(hardwareMap, device, "pinpoint");
            cached.add(new NamedPinpoint(name, device));
        }
    }

    private static String firstNameOrFallback(HardwareMap hardwareMap, HardwareDevice device, String fallback) {
        try {
            Set<String> names = hardwareMap.getNamesOf(device);
            if (names != null && !names.isEmpty()) {
                return names.iterator().next();
            }
        } catch (Throwable ignored) {
            // Some SDK variants may not support getNamesOf; fall back.
        }
        return fallback;
    }

    private static final class NamedPinpoint {
        private final String name;
        private final GoBildaPinpointDriver pinpoint;
        private final StructPoseInputs poses = new StructPoseInputs("Pose2d", "Pose3d");

        private NamedPinpoint(String name, GoBildaPinpointDriver pinpoint) {
            this.name = name;
            this.pinpoint = pinpoint;
        }
    }

    private final StructPoseInputs robotAliases = new StructPoseInputs("RobotPose", "RobotPose3d");
}
