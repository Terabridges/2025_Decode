package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;

@Configurable
@TeleOp(name = "TurretPositionCalibrator", group = "Test")
public class TurretPositionCalibrator extends OpMode {

    // Manual adjustment sizes
    public static double smallStepDeg = 1.0;
    public static double largeStepDeg = 5.0;
    public static boolean usePanelTarget = false;
    public static double panelTargetDeg = 180.0;
    public static double ref90TurretDeg = 59.25;
    public static double ref90EncoderDeg = 280.0;

    // Reference pose used to interpret BLUE/RED goal captures
    public static double testPoseX = 72.0;
    public static double testPoseY = 72.0;
    public static double testHeadingDeg = 90.0;

    // Field goal anchors (match turret lock defaults)
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 144.0;
    public static double redGoalX = 144.0;
    public static double redGoalY = 144.0;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private double targetDeg;
    private double captureMinDeg = Double.NaN;
    private double captureMaxDeg = Double.NaN;
    private double captureForwardDeg = Double.NaN;
    private double captureBlueGoalDeg = Double.NaN;
    private double captureRedGoalDeg = Double.NaN;
    private String lastCapture = "none";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
    }

    @Override
    public void start() {
        robot.outtake.turret.setAimLockEnabled(false);
        robot.outtake.turret.toInit();
        targetDeg = robot.outtake.turret.getCurrentDegrees();
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);

        if (usePanelTarget) {
            targetDeg = panelTargetDeg;
        }

        if (edge(current.dpad_right, previous.dpad_right)) targetDeg += smallStepDeg;
        if (edge(current.dpad_left, previous.dpad_left)) targetDeg -= smallStepDeg;
        if (edge(current.dpad_up, previous.dpad_up)) targetDeg = ref90TurretDeg;
        if (edge(current.right_bumper, previous.right_bumper)) targetDeg += largeStepDeg;
        if (edge(current.left_bumper, previous.left_bumper)) targetDeg -= largeStepDeg;

        if (edge(current.back, previous.back)) {
            robot.outtake.turret.toInit();
            targetDeg = robot.outtake.turret.getCurrentDegrees();
        }

        if (edge(current.a, previous.a)) {
            captureForwardDeg = robot.outtake.turret.getCurrentDegrees();
            lastCapture = "A: ROBOT_FORWARD";
        }
        if (edge(current.b, previous.b)) {
            captureMinDeg = robot.outtake.turret.getCurrentDegrees();
            lastCapture = "B: MIN_SAFE";
        }
        if (edge(current.start, previous.start)) {
            captureMaxDeg = robot.outtake.turret.getCurrentDegrees();
            lastCapture = "START: MAX_SAFE";
        }
        if (edge(current.x, previous.x)) {
            captureBlueGoalDeg = robot.outtake.turret.getCurrentDegrees();
            lastCapture = "X: BLUE_GOAL";
        }
        if (edge(current.y, previous.y)) {
            captureRedGoalDeg = robot.outtake.turret.getCurrentDegrees();
            lastCapture = "Y: RED_GOAL";
        }

        robot.outtake.turret.setAimLockEnabled(false);
        // Calibration mode: command raw servo position so soft turret limits do not clip travel.
        robot.outtake.turret.setTurretPos(targetDeg / 360.0);

        double currentDeg = robot.outtake.turret.getCurrentDegrees();
        double encoderDeg = robot.outtake.turret.getEncoderDegrees();
        double mappedEncoderTurretDeg = robot.outtake.turret.getMappedEncoderTurretDegrees();
        double encoderVoltage = robot.outtake.turret.getEncoderVoltage();
        double ref90TurretError = currentDeg - ref90TurretDeg;
        double ref90EncoderError = encoderDeg - ref90EncoderDeg;
        double ref90MappedError = robot.outtake.turret.getMappedEncoderErrorDeg(ref90TurretDeg);
        double expectedBlueRel = relativeGoalDeg(blueGoalX, blueGoalY);
        double expectedRedRel = relativeGoalDeg(redGoalX, redGoalY);

        joinedTelemetry.addData("Current Turret Deg", fmt(currentDeg));
        joinedTelemetry.addData("Target Deg", fmt(targetDeg));
        joinedTelemetry.addData("Encoder Deg", fmt(encoderDeg));
        joinedTelemetry.addData("Mapped Enc->Turret Deg", fmt(mappedEncoderTurretDeg));
        joinedTelemetry.addData("Encoder Voltage", fmt(encoderVoltage));
        joinedTelemetry.addData("Ref 90 Turret Deg", fmt(ref90TurretDeg));
        joinedTelemetry.addData("Ref 90 Encoder Deg", fmt(ref90EncoderDeg));
        joinedTelemetry.addData("Ref 90 Turret Err", fmt(ref90TurretError));
        joinedTelemetry.addData("Ref 90 Encoder Err", fmt(ref90EncoderError));
        joinedTelemetry.addData("Ref 90 Mapped Err", fmt(ref90MappedError));
        joinedTelemetry.addData("Panel Target Mode", usePanelTarget);
        joinedTelemetry.addData("Panel Target Deg", fmt(panelTargetDeg));
        joinedTelemetry.addData("Last Capture", lastCapture);

        joinedTelemetry.addData("CAP Forward (A)", fmt(captureForwardDeg));
        joinedTelemetry.addData("CAP Min Safe (B)", fmt(captureMinDeg));
        joinedTelemetry.addData("CAP Max Safe (START)", fmt(captureMaxDeg));
        joinedTelemetry.addData("CAP Blue Goal (X)", fmt(captureBlueGoalDeg));
        joinedTelemetry.addData("CAP Red Goal (Y)", fmt(captureRedGoalDeg));

        joinedTelemetry.addData("Expected Blue Rel @ Test Pose", fmt(expectedBlueRel));
        joinedTelemetry.addData("Expected Red Rel @ Test Pose", fmt(expectedRedRel));

        if (!Double.isNaN(captureMinDeg) && !Double.isNaN(captureMaxDeg)) {
            joinedTelemetry.addData("Suggested turretMinDeg", fmt(Math.min(captureMinDeg, captureMaxDeg)));
            joinedTelemetry.addData("Suggested turretMaxDeg", fmt(Math.max(captureMinDeg, captureMaxDeg)));
        }

        joinedTelemetry.addLine("Controls:");
        joinedTelemetry.addLine("Dpad L/R: +/- small, Dpad UP: goto Ref 90, LB/RB: +/- large, BACK: toInit");
        joinedTelemetry.addLine("A=Capture FORWARD, B=Capture MIN, START=Capture MAX");
        joinedTelemetry.addLine("X=Capture BLUE goal, Y=Capture RED goal");
        joinedTelemetry.addLine("Panels: set usePanelTarget=true and panelTargetDeg to command directly.");
        joinedTelemetry.addLine("Direction check: press Dpad Right (+deg), observe robot-left or robot-right.");
        joinedTelemetry.update();
    }

    private boolean edge(boolean now, boolean prevVal) {
        return now && !prevVal;
    }

    private double relativeGoalDeg(double gx, double gy) {
        double headingToGoalDeg = Math.toDegrees(Math.atan2(gy - testPoseY, gx - testPoseX));
        return normalizeDeg(headingToGoalDeg - testHeadingDeg);
    }

    private double normalizeDeg(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    private String fmt(double v) {
        if (Double.isNaN(v)) return "unset";
        return String.format("%.2f", v);
    }
}
