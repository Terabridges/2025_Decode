package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight_subsystem {

    // ---- Pipelines (match your Limelight UI slots) ----
    public static final int PIPE_APRILTAG   = 0;
    public static final int PIPE_GAMEPIECE  = 1;

    // Thin snapshot for easy consumption
    public static class TargetInfo {
        public final boolean valid;
        public final double txDeg, tyDeg, areaPct;
        public final long stalenessMs;
        public TargetInfo(boolean v, double tx, double ty, double a, long s) {
            valid=v; txDeg=tx; tyDeg=ty; areaPct=a; stalenessMs=s;
        }
        public static TargetInfo empty(){ return new TargetInfo(false,0,0,0,Long.MAX_VALUE); }
    }

    private Limelight3A ll;
    private int currentPipe = PIPE_APRILTAG;

    // Tuning knobs
    private int pollHz = 100;
    private long maxStalenessMs = 120;
    private double aimKP = 0.03;

    // MT2 support: keep last IMU yaw (deg) and push it to Limelight each loop (from your OpMode)
    private double lastYawDeg = 0.0;

    // ---- Lifecycle ----
    public void init(HardwareMap hw, String deviceName) {
        ll = hw.get(Limelight3A.class, deviceName); // e.g., "limelight"
        ll.setPollRateHz(pollHz);
    }

    public void start() {
        if (ll != null) ll.start();
        pipelineSwitch(currentPipe);
    }

    public void stop() {
        if (ll != null) ll.stop();
    }

    public void pipelineSwitch(int index) {
        currentPipe = index;
        if (ll != null) ll.pipelineSwitch(index);
    }

    // ---- Targets ----
    public TargetInfo getTarget() {
        if (ll == null) return TargetInfo.empty();
        LLResult r = ll.getLatestResult();
        if (r == null || !r.isValid()) return TargetInfo.empty();
        long stale = r.getStaleness();
        if (stale > maxStalenessMs) return TargetInfo.empty();
        return new TargetInfo(true, r.getTx(), r.getTy(), r.getTa(), stale);
    }

    /** Simple aim assist: strafe command [-1..1] to center tx≈0. */
    public double aimStrafeCommand() {
        TargetInfo t = getTarget();
        if (!t.valid) return 0.0;
        return clamp(t.txDeg * aimKP, -0.6, 0.6);
    }

    // ---- Pose (MT1 & MT2) ----

    /** Pose from MegaTag1 (AprilTags only). Returns null if invalid/stale. */
    public Pose3D getBotPoseMT1() {
        if (ll == null) return null;
        LLResult r = ll.getLatestResult();
        if (r == null || !r.isValid() || r.getStaleness() > maxStalenessMs) return null;
        return r.getBotpose(); // Pose3D
    }

    /**
     * Pose from MegaTag2 (LL+IMU). Call updateImuYawDeg(...) each loop before reading.
     * Returns null if invalid/stale.
     */
    public Pose3D getBotPoseMT2() {
        if (ll == null) return null;
        // Push robot yaw (deg) to Limelight each loop BEFORE calling getLatestResult()
        ll.updateRobotOrientation(lastYawDeg);
        LLResult r = ll.getLatestResult();
        if (r == null || !r.isValid() || r.getStaleness() > maxStalenessMs) return null;
        return r.getBotpose_MT2(); // Pose3D
    }

    /** Provide IMU yaw (deg, field-agnostic) every loop for MT2 fusion. */
    public void updateImuYawDeg(double yawDeg) { this.lastYawDeg = yawDeg; }

    // Convenience getters (null-safe: return NaN if no pose)
    public double getPoseX_Mt1() { Pose3D p = getBotPoseMT1(); return (p==null)?Double.NaN:p.getPosition().x; }
    public double getPoseY_Mt1() { Pose3D p = getBotPoseMT1(); return (p==null)?Double.NaN:p.getPosition().y; }
    public double getYawRad_Mt1(){ Pose3D p = getBotPoseMT1(); return (p==null)?Double.NaN:p.getOrientation().getYaw(AngleUnit.RADIANS); }

    public double getPoseX_Mt2() { Pose3D p = getBotPoseMT2(); return (p==null)?Double.NaN:p.getPosition().x; }
    public double getPoseY_Mt2() { Pose3D p = getBotPoseMT2(); return (p==null)?Double.NaN:p.getPosition().y; }
    public double getYawRad_Mt2(){ Pose3D p = getBotPoseMT2(); return (p==null)?Double.NaN:p.getOrientation().getYaw(AngleUnit.RADIANS); }

    // ---- Tuning hooks ----
    public int  getCurrentPipeline() { return currentPipe; }
    public void setMaxStalenessMs(long ms) { maxStalenessMs = ms; }
    public void setAimKP(double kp) { aimKP = kp; }
    public void setPollHz(int hz) { pollHz = hz; if (ll != null) ll.setPollRateHz(hz); }

    // ---- Helpers ----
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

