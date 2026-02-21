package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

@Configurable
@TeleOp(name = "TurretFrameCharacterization", group = "Test")
public class TurretFrameCharacterization extends OpMode {

    public static double smallStepDeg = 2.0;
    public static double largeStepDeg = 10.0;
    public static double settleThresholdDeg = 1.0;

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    private double targetDeg = 0.0;
    private double previousEncoderDeg = 0.0;
    private String lastAction = "none";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void start() {
        robot.outtake.turret.toInit();
        robot.outtake.turret.setAimLockEnabled(false);
        targetDeg = robot.outtake.turret.getCurrentDegrees();
        previousEncoderDeg = robot.outtake.turret.getEncoderDegrees();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (edge(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) {
            targetDeg += smallStepDeg;
            lastAction = "Dpad Right (+small)";
        }
        if (edge(currentGamepad1.dpad_left, previousGamepad1.dpad_left)) {
            targetDeg -= smallStepDeg;
            lastAction = "Dpad Left (-small)";
        }
        if (edge(currentGamepad1.right_bumper, previousGamepad1.right_bumper)) {
            targetDeg += largeStepDeg;
            lastAction = "Right Bumper (+large)";
        }
        if (edge(currentGamepad1.left_bumper, previousGamepad1.left_bumper)) {
            targetDeg -= largeStepDeg;
            lastAction = "Left Bumper (-large)";
        }

        if (edge(currentGamepad1.a, previousGamepad1.a)) {
            targetDeg = 0.0;
            lastAction = "Preset A -> 0 deg";
        }
        if (edge(currentGamepad1.b, previousGamepad1.b)) {
            targetDeg = 90.0;
            lastAction = "Preset B -> 90 deg";
        }
        if (edge(currentGamepad1.x, previousGamepad1.x)) {
            targetDeg = 180.0;
            lastAction = "Preset X -> 180 deg";
        }
        if (edge(currentGamepad1.y, previousGamepad1.y)) {
            targetDeg = 270.0;
            lastAction = "Preset Y -> 270 deg";
        }

        if (edge(currentGamepad1.start, previousGamepad1.start)) {
            targetDeg = robot.outtake.turret.getEncoderDegrees();
            lastAction = "Start -> target := encoder";
        }
        if (edge(currentGamepad1.back, previousGamepad1.back)) {
            robot.outtake.turret.toInit();
            targetDeg = robot.outtake.turret.getCurrentDegrees();
            lastAction = "Back -> turret init";
        }

        robot.outtake.turret.setAimLockEnabled(false);
        robot.outtake.turret.setTurretDegree(targetDeg);

        double commandedDeg = robot.outtake.turret.getCurrentDegrees();
        double encoderDeg = robot.outtake.turret.getEncoderDegrees();
        double encoderDelta = wrapSignedDeg(encoderDeg - previousEncoderDeg);
        double trackingError = wrapSignedDeg(encoderDeg - commandedDeg);

        joinedTelemetry.addData("Target Deg", targetDeg);
        joinedTelemetry.addData("Servo Command Deg", commandedDeg);
        joinedTelemetry.addData("Encoder Deg", encoderDeg);
        joinedTelemetry.addData("Encoder Delta/Loop", encoderDelta);
        joinedTelemetry.addData("Tracking Error (enc-cmd)", trackingError);
        joinedTelemetry.addData("Encoder Voltage", robot.outtake.turret.getEncoderVoltage());
        joinedTelemetry.addData("Last Action", lastAction);
        joinedTelemetry.addData("At Target?", Math.abs(trackingError) <= settleThresholdDeg);

        joinedTelemetry.addLine("Controls:");
        joinedTelemetry.addLine("Dpad L/R: small step | LB/RB: large step");
        joinedTelemetry.addLine("A/B/X/Y: presets 0/90/180/270");
        joinedTelemetry.addLine("START: set target=encoder | BACK: toInit");
        joinedTelemetry.addLine("Observe direction: press Dpad Right (+deg), note turret goes robot-left or robot-right.");
        joinedTelemetry.update();

        previousEncoderDeg = encoderDeg;
    }

    private boolean edge(boolean now, boolean prev) {
        return now && !prev;
    }

    private double wrapSignedDeg(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
