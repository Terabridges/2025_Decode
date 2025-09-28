package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision implements Subsystem{

    //---------------- Hardware ----------------
    public NormalizedColorSensor ballInspector;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    //---------------- Software ----------------
    public LLResult latest; //Cached result each loop
    public int currentPipeline = 0; //Current pipeline index (0..9)

    //---------------- Constructor ----------------
    public Vision(HardwareMap map) {
        ballInspector = map.get(NormalizedColorSensor.class, "ball_inspector");
        limelight = map.get(Limelight3A.class, "limelight");
        pinpoint =  map.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    //---------------- Methods ----------------
    private void limelightInit(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
    }

    private void limelightUpdate(){
        if (pinpoint != null) {
            double yawDeg = (pinpoint.getHeading(AngleUnit.DEGREES)-90);
            limelight.updateRobotOrientation(yawDeg);
        }

        latest = limelight.getLatestResult();

        if (latest != null && latest.isValid()) {
            Pose3D botpose = latest.getBotpose_MT2();
        }
    }

    public void pipeline(int index) {
        currentPipeline = Range.clip(index, 0, 9);
        if (limelight != null) limelight.pipelineSwitch(currentPipeline);
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

    public void pinpointInit(){
        pinpoint.setOffsets(0, 0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void pinpointUpdate(){
        pinpoint.update();
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){
        limelightInit();
        pinpointInit();
    }

    @Override
    public void update(){
        limelightUpdate();
        pinpointUpdate();
    }

}
