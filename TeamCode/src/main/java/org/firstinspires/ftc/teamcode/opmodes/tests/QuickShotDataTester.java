package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.config.subsystems.Vision;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "QuickShotDataTester", group = "Test")
@PsiKitAutoLog(rlogPort = 5802)
public class QuickShotDataTester extends OpMode {

    private static final int MAX_DATA_POINTS = 200;
    private static final int MAX_ROWS_SHOWN = 12;
    private static final double[] RPM_INCREMENTS = {50, 100, 250, 500, 1000};
    private static final double[] HOOD_INCREMENTS = {0.01, 0.025, 0.05, 0.1};
    private static final String DATA_FILE_NAME = "quick_shot_data.csv";
    private static final double TICKS_PER_REV = 28.0;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx flyLeft;
    private DcMotorEx flyRight;
    private Servo hood;
    private Vision vision;

    private boolean hasDrive;
    private boolean hasShooter;
    private boolean hasHood;
    private boolean hasVision;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    private final List<DataPoint> dataPoints = new ArrayList<>();
    private int nextDataId = 1;
    private int rpmStepIndex = 3;
    private int hoodStepIndex = 1;
    private double hoodTarget = 0.5;
    private double targetRPM = 0.0;
    private boolean shooterOn = false;
    private File dataFile;
    private String saveStatus = "Not saved yet";

    @Override
    public void init() {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        initDriveHardware();
        initShooterHardware();
        initVisionHardware();

        File firstFolder = new File("/sdcard/FIRST");
        if (!firstFolder.exists()) {
            //noinspection ResultOfMethodCallIgnored
            firstFolder.mkdirs();
        }
        dataFile = new File(firstFolder, DATA_FILE_NAME);
        saveDataToFile();
    }

    @Override
    public void init_loop() {
        gamepadUpdate();
        telemetry.addData("QuickShotDataTester", "Ready");
        telemetry.addData("Drive", "LS move, RSX turn");
        telemetry.addData("Shooter Toggle", "LB");
        telemetry.addData("RPM +/-", "Dpad Up/Down");
        telemetry.addData("RPM Step +/-", "Dpad Right/Left");
        telemetry.addData("Hood +/-", "Y/A");
        telemetry.addData("Hood Step Cycle", "X");
        telemetry.addData("Apply Hood Target", "RB");
        telemetry.addData("Capture Data Point", "Start");
        telemetry.addData("Clear Data", "Back");
        telemetry.addData("File", dataFile.getAbsolutePath());
        telemetry.addData("Drive Hardware", hasDrive ? "OK" : "Missing");
        telemetry.addData("Shooter Hardware", hasShooter ? "OK" : "Missing");
        telemetry.addData("Hood Hardware", hasHood ? "OK" : "Missing");
        telemetry.addData("Vision Hardware", hasVision ? "OK" : "Missing");
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepadUpdate();

        updateDriveControl();
        updateShooterControl();
        updateHoodControl();
        updateOutputs();

        if (currentGamepad1.start && !previousGamepad1.start) {
            captureDataPoint();
        }
        if (currentGamepad1.back && !previousGamepad1.back) {
            dataPoints.clear();
            nextDataId = 1;
            saveDataToFile();
        }

        addTelemetry();
    }

    @Override
    public void stop() {
        setDrivePowers(0, 0, 0, 0);
        if (flyLeft != null) flyLeft.setVelocity(0);
        if (flyRight != null) flyRight.setVelocity(0);
    }

    private void initDriveHardware() {
        leftFront = getOptionalDcMotor("left_front", "leftFront", "lf");
        rightFront = getOptionalDcMotor("right_front", "rightFront", "rf");
        leftBack = getOptionalDcMotor("left_back", "leftBack", "lb");
        rightBack = getOptionalDcMotor("right_back", "rightBack", "rb");

        hasDrive = leftFront != null && rightFront != null && leftBack != null && rightBack != null;
        if (hasDrive) {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initShooterHardware() {
        flyLeft = getOptionalDcMotorEx("fly_left", "flyLeft", "flywheel_left", "shooter_left");
        flyRight = getOptionalDcMotorEx("fly_right", "flyRight", "flywheel_right", "shooter_right");
        hood = getOptionalServo("hood", "hood_servo");

        hasShooter = flyLeft != null && flyRight != null;
        hasHood = hood != null;

        if (hasShooter) {
            flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            flyRight.setDirection(DcMotorSimple.Direction.FORWARD);
            flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (hasHood) {
            hood.setDirection(Servo.Direction.FORWARD);
            hood.setPosition(hoodTarget);
        }
    }

    private void initVisionHardware() {
        try {
            vision = new Vision(hardwareMap);
            vision.toInit();
            hasVision = true;
        } catch (Exception e) {
            vision = null;
            hasVision = false;
        }
    }

    private void updateDriveControl() {
        if (!hasDrive) return;

        double forward = -currentGamepad1.left_stick_y;
        double strafe = currentGamepad1.left_stick_x;
        double rotate = currentGamepad1.right_stick_x;

        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        setDrivePowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void updateShooterControl() {
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            shooterOn = !shooterOn;
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            targetRPM += RPM_INCREMENTS[rpmStepIndex];
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            targetRPM -= RPM_INCREMENTS[rpmStepIndex];
            if (targetRPM < 0) targetRPM = 0;
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            rpmStepIndex = (rpmStepIndex + 1) % RPM_INCREMENTS.length;
        }
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            rpmStepIndex = (rpmStepIndex - 1 + RPM_INCREMENTS.length) % RPM_INCREMENTS.length;
        }
    }

    private void updateHoodControl() {
        if (currentGamepad1.y && !previousGamepad1.y) {
            hoodTarget += HOOD_INCREMENTS[hoodStepIndex];
        }
        if (currentGamepad1.a && !previousGamepad1.a) {
            hoodTarget -= HOOD_INCREMENTS[hoodStepIndex];
        }
        hoodTarget = clamp(hoodTarget, 0.0, 1.0);

        if (currentGamepad1.x && !previousGamepad1.x) {
            hoodStepIndex = (hoodStepIndex + 1) % HOOD_INCREMENTS.length;
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && hood != null) {
            hood.setPosition(hoodTarget);
        }
    }

    private void updateOutputs() {
        if (hasShooter) {
            double targetVel = rpmToVelocity(shooterOn ? targetRPM : 0.0);
            flyLeft.setVelocity(targetVel);
            flyRight.setVelocity(targetVel);
        }
        if (hasVision && vision != null) {
            vision.update();
        }
    }

    private void captureDataPoint() {
        if (dataPoints.size() >= MAX_DATA_POINTS) {
            dataPoints.remove(0);
        }

        DataPoint point = new DataPoint(
                nextDataId++,
                getRuntime(),
                getDistanceInches(),
                targetRPM,
                getCurrentRPM(),
                hoodTarget,
                getHoodActual(),
                getTagId(),
                hasTarget()
        );
        dataPoints.add(point);
        saveDataToFile();
    }

    private void saveDataToFile() {
        try {
            StringBuilder csv = new StringBuilder();
            csv.append("id,time_sec,distance_in,target_rpm,current_rpm,hood_target,hood_actual,tag_id,has_target\n");
            for (DataPoint p : dataPoints) {
                csv.append(String.format(
                        Locale.US,
                        "%d,%.3f,%.3f,%.1f,%.1f,%.4f,%.4f,%d,%s\n",
                        p.id, p.timeSec, p.distanceInches, p.targetRpm, p.currentRpm,
                        p.hoodTarget, p.hoodActual, p.tagId, p.hasTarget ? "true" : "false"
                ));
            }
            ReadWriteFile.writeFile(dataFile, csv.toString());
            saveStatus = "Saved " + dataPoints.size() + " rows";
        } catch (Exception e) {
            saveStatus = "Save failed: " + e.getMessage();
        }
    }

    private void addTelemetry() {
        telemetry.addData("Distance (in)", "%.2f", getDistanceInches());
        telemetry.addData("Tag ID", getTagId());
        telemetry.addData("Has Target", hasTarget());
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", getCurrentRPM());
        telemetry.addData("Hood Target", "%.4f", hoodTarget);
        telemetry.addData("Hood Actual", "%.4f", getHoodActual());
        telemetry.addData("RPM Step", "%.0f", RPM_INCREMENTS[rpmStepIndex]);
        telemetry.addData("Hood Step", "%.3f", HOOD_INCREMENTS[hoodStepIndex]);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Saved Points", dataPoints.size());
        telemetry.addData("Save Status", saveStatus);
        telemetry.addData("File", dataFile.getAbsolutePath());
        telemetry.addData("Drive Hardware", hasDrive ? "OK" : "Missing");
        telemetry.addData("Shooter Hardware", hasShooter ? "OK" : "Missing");
        telemetry.addData("Hood Hardware", hasHood ? "OK" : "Missing");
        telemetry.addData("Vision Hardware", hasVision ? "OK" : "Missing");

        telemetry.addLine("Table: # | t | dist | hood | rpm | tag");
        int start = Math.max(0, dataPoints.size() - MAX_ROWS_SHOWN);
        for (int i = start; i < dataPoints.size(); i++) {
            DataPoint p = dataPoints.get(i);
            telemetry.addLine(String.format(
                    Locale.US,
                    "%03d | %.1f | %.1f | %.3f | %.0f | %d",
                    p.id, p.timeSec, p.distanceInches, p.hoodActual, p.currentRpm, p.tagId
            ));
        }

        telemetry.update();
    }

    private void gamepadUpdate() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
    }

    private void setDrivePowers(double lf, double rf, double lb, double rb) {
        if (!hasDrive) return;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    private double getDistanceInches() {
        if (!hasVision || vision == null) return 0.0;
        return vision.getDistanceInches();
    }

    private int getTagId() {
        if (!hasVision || vision == null) return -1;
        return vision.getCurrentTagId();
    }

    private boolean hasTarget() {
        return hasVision && vision != null && vision.hasTarget();
    }

    private double getHoodActual() {
        return hood != null ? hood.getPosition() : 0.0;
    }

    private double getCurrentRPM() {
        DcMotorEx rpmMotor = flyLeft;
        if (rpmMotor == null) return 0.0;
        return velocityToRPM(rpmMotor.getVelocity());
    }

    private double velocityToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    private double rpmToVelocity(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private DcMotor getOptionalDcMotor(String... names) {
        for (String name : names) {
            try {
                return hardwareMap.get(DcMotor.class, name);
            } catch (Exception ignored) {
            }
        }
        return null;
    }

    private DcMotorEx getOptionalDcMotorEx(String... names) {
        for (String name : names) {
            try {
                return hardwareMap.get(DcMotorEx.class, name);
            } catch (Exception ignored) {
            }
        }
        return null;
    }

    private Servo getOptionalServo(String... names) {
        for (String name : names) {
            try {
                return hardwareMap.get(Servo.class, name);
            } catch (Exception ignored) {
            }
        }
        return null;
    }

    private static class DataPoint {
        final int id;
        final double timeSec;
        final double distanceInches;
        final double targetRpm;
        final double currentRpm;
        final double hoodTarget;
        final double hoodActual;
        final int tagId;
        final boolean hasTarget;

        DataPoint(int id,
                  double timeSec,
                  double distanceInches,
                  double targetRpm,
                  double currentRpm,
                  double hoodTarget,
                  double hoodActual,
                  int tagId,
                  boolean hasTarget) {
            this.id = id;
            this.timeSec = timeSec;
            this.distanceInches = distanceInches;
            this.targetRpm = targetRpm;
            this.currentRpm = currentRpm;
            this.hoodTarget = hoodTarget;
            this.hoodActual = hoodActual;
            this.tagId = tagId;
            this.hasTarget = hasTarget;
        }
    }
}
