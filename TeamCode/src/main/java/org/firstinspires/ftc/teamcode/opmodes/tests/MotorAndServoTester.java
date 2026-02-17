package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

@TeleOp(name = "MotorAndServoTester", group = "Test")
@PsiKitAutoLog(rlogPort = 5802)
public class MotorAndServoTester extends LinearOpMode {

    private List<DcMotor> motors = new ArrayList<>();
    private List<String> motorNames = new ArrayList<>();
    private List<Servo> servos = new ArrayList<>();
    private List<String> servoNames = new ArrayList<>();
    private List<CRServo> crServos = new ArrayList<>();
    private List<String> crServoNames = new ArrayList<>();

    private int motorIndex = 0;
    private int servoIndex = 0;
    private int crServoIndex = 0;
    private int testMode = 0; // 0: motors, 1: servos, 2: crservos

    private boolean powerCapEnabled = true;
    private boolean prevX = false;
    private static final double CAPPED_POWER_SCALE = 0.5;

    private boolean servoActive = false;
    private boolean prevB = false;
    private int lastServoIndex = -1;

    private boolean crServoActive = false;
    private int lastCrServoIndex = -1;

    private boolean bEdge = false;

    @Override
    public void runOpMode() {
        // Get all devices and their names
        motors = hardwareMap.getAll(DcMotor.class);
        servos = hardwareMap.getAll(Servo.class);
        crServos = hardwareMap.getAll(CRServo.class);

        motorNames = getNamesForDevices(motors);
        servoNames = getNamesForDevices(servos);
        crServoNames = getNamesForDevices(crServos);

        telemetry.addData("Motors Found", motors.size());
        telemetry.addData("Servos Found", servos.size());
        telemetry.addData("CRServos Found", crServos.size());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Cycle test modes with Y button (motors -> servos -> crservos -> motors)
            if (gamepad1.y) {
                testMode = (testMode + 1) % 3;
                sleep(200); // Debounce
            }

            // Toggle power cap with X (edge-detected)
            if (gamepad1.x && !prevX) {
                powerCapEnabled = !powerCapEnabled;
            }
            prevX = gamepad1.x;

            // Edge-detect B once per loop so it can be reused across modes.
            bEdge = gamepad1.b && !prevB;
            prevB = gamepad1.b;

            addControlsLegend();

            if (testMode == 0) {
                testMotors();
            } else if (testMode == 1) {
                testServos();
            } else {
                testCRServos();
            }

            telemetry.update();
        }
    }

    private void addControlsLegend() {
        telemetry.addLine("Controls:");
        telemetry.addLine("  Y: switch mode (Motor/Servo/CRServo)");
        telemetry.addLine("  Dpad Left/Right: select device");

        telemetry.addLine(String.format(
                "  X: toggle power cap (currently %s)",
                powerCapEnabled ? "ON (0.5x)" : "OFF (1.0x)"
        ));

        if (testMode == 0) {
            telemetry.addLine("  Left stick Y: motor power (scaled)");
            telemetry.addLine("  A: stop motor");
        } else if (testMode == 1) {
            telemetry.addLine("  Right stick X: servo position (0..1)");
            telemetry.addLine("  B: toggle servo active (PWM enable/disable)");
            telemetry.addLine("  A: center servo (0.5)");
        } else {
            telemetry.addLine("  Left stick Y: CRServo power (scaled)");
            telemetry.addLine("  B: toggle CRServo active (PWM enable/disable)");
            telemetry.addLine("  A: stop CRServo");
        }
    }

    private List<String> getNamesForDevices(List<? extends HardwareDevice> devices) {
        List<String> names = new ArrayList<>();
        for (HardwareDevice device : devices) {
            String name = getDeviceName(device);
            names.add(name != null ? name : "Unnamed");
        }
        return names;
    }

    private String getDeviceName(HardwareDevice device) {
        try {
            Set<String> names = hardwareMap.getNamesOf(device);
            if (names != null && !names.isEmpty()) {
                return names.iterator().next();
            }
        } catch (Throwable ignored) {
            // Some SDK variants may not support getNamesOf.
        }
        return null; // caller provides fallback
    }

    private void testMotors() {
        // Cycle through motors with D-pad left/right
        if (gamepad1.dpad_right && motorIndex < motors.size() - 1) {
            motorIndex++;
            sleep(200);
        } else if (gamepad1.dpad_left && motorIndex > 0) {
            motorIndex--;
            sleep(200);
        }

        if (motors.size() > 0) {
            DcMotor currentMotor = motors.get(motorIndex);
            String name = motorNames.get(motorIndex);
            // Set power based on left stick Y (range -1 to 1, cap at 0.5 for safety)
            double maxPower = powerCapEnabled ? CAPPED_POWER_SCALE : 1.0;
            double power = -gamepad1.left_stick_y * maxPower;
            currentMotor.setPower(power);

            // Reset to zero with A button
            if (gamepad1.a) {
                currentMotor.setPower(0);
            }

            telemetry.addData("Mode", "Motor");
            telemetry.addData("Testing Motor", motorIndex + "/" + (motors.size() - 1) + " (" + name + ")");
            telemetry.addData("Power", power);
            telemetry.addData("Power Cap", powerCapEnabled ? "ON (0.5x)" : "OFF (1.0x)");
        } else {
            telemetry.addData("Mode", "Motor");
            telemetry.addData("No Motors", "Connect some!");
        }
    }

    private void testServos() {
        // Cycle through servos with D-pad left/right
        if (gamepad1.dpad_right && servoIndex < servos.size() - 1) {
            servoIndex++;
            sleep(200);
        } else if (gamepad1.dpad_left && servoIndex > 0) {
            servoIndex--;
            sleep(200);
        }

        if (servos.size() > 0) {
            // If selection changed, disable PWM on the previously-selected servo and reset active state.
            if (lastServoIndex != servoIndex) {
                if (lastServoIndex >= 0 && lastServoIndex < servos.size()) {
                    disableServoPwm(servos.get(lastServoIndex));
                }
                servoActive = false;
                lastServoIndex = servoIndex;
            }

            Servo currentServo = servos.get(servoIndex);
            String name = servoNames.get(servoIndex);

            // Toggle servo active (PWM enabled) with B.
            if (bEdge) {
                servoActive = !servoActive;
                if (!servoActive) {
                    disableServoPwm(currentServo);
                }
            }

            // Set position based on right stick X (range 0..1) but only when active.
            double position = (gamepad1.right_stick_x + 1) / 2.0;
            if (servoActive) {
                enableServoPwm(currentServo);
                currentServo.setPosition(position);
            }

            // Reset to 0.5 with A button
            if (gamepad1.a) {
                currentServo.setPosition(0.5);
            }

            telemetry.addData("Mode", "Servo");
            telemetry.addData("Testing Servo", servoIndex + "/" + (servos.size() - 1) + " (" + name + ")");
            telemetry.addData("Position", position);
            telemetry.addData("Active", servoActive ? "YES (PWM enabled)" : "NO (PWM disabled)");
        } else {
            telemetry.addData("Mode", "Servo");
            telemetry.addData("No Servos", "Connect some!");
        }
    }

    private void enableServoPwm(Servo servo) {
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmEnable();
        }
    }

    private void disableServoPwm(Servo servo) {
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmDisable();
        }
    }

    private void testCRServos() {
        // Cycle through crservos with D-pad left/right
        if (gamepad1.dpad_right && crServoIndex < crServos.size() - 1) {
            crServoIndex++;
            sleep(200);
        } else if (gamepad1.dpad_left && crServoIndex > 0) {
            crServoIndex--;
            sleep(200);
        }

        if (crServos.size() > 0) {
            // If selection changed, stop the previously-selected CRServo and reset active state.
            if (lastCrServoIndex != crServoIndex) {
                if (lastCrServoIndex >= 0 && lastCrServoIndex < crServos.size()) {
                    disablePwmIfSupported(crServos.get(lastCrServoIndex));
                }
                crServoActive = false;
                lastCrServoIndex = crServoIndex;
            }

            CRServo currentCRServo = crServos.get(crServoIndex);
            String name = crServoNames.get(crServoIndex);

            // Toggle CRServo active with B. When toggled off, ensure output is stopped.
            if (bEdge) {
                crServoActive = !crServoActive;
                if (!crServoActive) {
                    disablePwmIfSupported(currentCRServo);
                }
            }

            // Set power based on left stick Y (range -1 to 1, cap at 0.5 for safety)
            double maxPower = powerCapEnabled ? CAPPED_POWER_SCALE : 1.0;
            double requestedPower = -gamepad1.left_stick_y * maxPower;
            Double appliedPower = null;
            if (crServoActive) {
                enablePwmIfSupported(currentCRServo);
                currentCRServo.setPower(requestedPower);
                appliedPower = requestedPower;
            }

            // Reset to zero with A button
            if (gamepad1.a) {
                if (crServoActive) {
                    enablePwmIfSupported(currentCRServo);
                    currentCRServo.setPower(0);
                    appliedPower = 0.0;
                }
            }

            telemetry.addData("Mode", "CRServo");
            telemetry.addData("Testing CRServo", crServoIndex + "/" + (crServos.size() - 1) + " (" + name + ")");
            telemetry.addData("Power", appliedPower == null ? "(inactive)" : appliedPower);
            telemetry.addData("Power Cap", powerCapEnabled ? "ON (0.5x)" : "OFF (1.0x)");
            telemetry.addData("Active", crServoActive ? "YES" : "NO");
        } else {
            telemetry.addData("Mode", "CRServo");
            telemetry.addData("No CRServos", "Connect some!");
        }
    }

    private void enablePwmIfSupported(HardwareDevice device) {
        if (device instanceof PwmControl) {
            ((PwmControl) device).setPwmEnable();
        }
    }

    private void disablePwmIfSupported(HardwareDevice device) {
        if (device instanceof PwmControl) {
            ((PwmControl) device).setPwmDisable();
        }
    }
}
