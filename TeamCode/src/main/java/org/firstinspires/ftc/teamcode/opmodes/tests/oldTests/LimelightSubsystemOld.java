package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Subsystem;

public class LimelightSubsystemOld implements Subsystem {

    /** Config names */
    public String limelightName = "limelight";
    public String imuName = "imu"; // set null/"" to disable MT2 yaw fusion

    /** Hardware devices */
    public Limelight3A camera;
    public BNO055IMU imu;

    // -----------------------------------------------------

    private HardwareMap hw;
    private Telemetry tele;
    public LLResult latest; //Cached result each loop
    public int currentPipeline = 0; //Current pipeline index (0..9)

    public LimelightSubsystemOld(HardwareMap hw, Telemetry tele) {
        this.hw = hw;
        this.tele = tele;
    }

    // ------------ Subsystem lifecycle ------------
    @Override
    public void toInit() {
        camera = hw.get(Limelight3A.class, limelightName);

        if (imuName != null && !imuName.isEmpty()) {
            imu = hw.get(BNO055IMU.class, imuName);
        }

        camera.pipelineSwitch(currentPipeline);
        camera.start();
    }

    @Override
    public void update() {

        if (imu != null) {
            double yawDeg = imu.getAngularOrientation().firstAngle; // FTC default yaw (Z)
            camera.updateRobotOrientation(yawDeg);
        }

        latest = camera.getLatestResult();

        if (latest != null && latest.isValid()) {
            Pose3D botpose = latest.getBotpose_MT2();
        }

        if (tele != null) {
            boolean valid = latest != null && latest.isValid();
            tele.addData("LL valid", valid);
            if (valid) {
                tele.addData("LL Tx", latest.getTx());
                tele.addData("LL Ty", latest.getTy());
            }
        }
    }

    /** HELPER FUNCTIONS */

    public void pipeline(int index) {
        currentPipeline = Range.clip(index, 0, 9);
        if (camera != null) camera.pipelineSwitch(currentPipeline);
    }

    public boolean hasTarget() { return latest != null && latest.isValid(); }

    public double getTx(double def) {
        if (hasTarget()) {
            try { return latest.getTx(); } catch (Throwable ignored) {}
        }
        return def;
    }

    public double getTy(double def) {
        if (hasTarget()) {
            try { return latest.getTy(); } catch (Throwable ignored) {}
        }
        return def;
    }

    public Pose3D getBotPoseMT2() {
        if (hasTarget()) {
            try { return latest.getBotpose_MT2(); } catch (Throwable ignored) {}
        }
        return null;
    }

    public Pose3D getBotPoseMT1() {
        if (hasTarget()) {
            try { return latest.getBotpose(); } catch (Throwable ignored) {}
        }
        return null;
    }
}
