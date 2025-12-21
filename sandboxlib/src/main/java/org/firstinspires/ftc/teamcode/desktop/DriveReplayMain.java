package org.firstinspires.ftc.teamcode.desktop;

import org.firstinspires.ftc.teamcode.config.control.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.GamepadView;
import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.io.DriveIO;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;

/**
 * Minimal desktop entrypoint to prove the IO pattern works off-robot.
 *
 * This does NOT run an FTC OpMode; it just replays a log file and steps logic.
 */
public final class DriveReplayMain {

    private static final class DriveIOReplay implements DriveIO {
        private long tick;

        @Override
        public void updateInputs(Inputs inputs) {
            // No sensors used by Drive yet.
        }

        @Override
        public void setMotorPowers(double lf, double rf, double lb, double rb) {
            // For now, just print what Drive wants to do.
            if (tick < 20 || tick % 50 == 0) {
                System.out.printf("Applied powers: lf=%.3f rf=%.3f lb=%.3f rb=%.3f%n", lf, rf, lb, rb);
            }
        }

        public void nextTick() {
            tick++;
        }
    }

    private static final class ReplayGamepad implements GamepadView {
        private double leftStickX;
        private double leftStickY;
        private double rightStickX;
        private boolean dpadUp;
        private boolean dpadDown;
        private boolean dpadLeft;
        private boolean dpadRight;
        private boolean leftBumper;
        private boolean rightBumper;
        private double leftTrigger;
        private double rightTrigger;
        private boolean a;
        private boolean b;
        private boolean x;
        private boolean y;
        private boolean leftStickButton;
        private boolean rightStickButton;

        public void setLeftStickX(double value) {
            leftStickX = value;
        }

        public void setLeftStickY(double value) {
            leftStickY = value;
        }

        public void setRightStickX(double value) {
            rightStickX = value;
        }

        public void setDpadDown(boolean value) {
            dpadDown = value;
        }

        @Override
        public double leftStickX() {
            return leftStickX;
        }

        @Override
        public double leftStickY() {
            return leftStickY;
        }

        @Override
        public double rightStickX() {
            return rightStickX;
        }

        @Override
        public boolean dpadUp() {
            return dpadUp;
        }

        @Override
        public boolean dpadDown() {
            return dpadDown;
        }

        @Override
        public boolean dpadLeft() {
            return dpadLeft;
        }

        @Override
        public boolean dpadRight() {
            return dpadRight;
        }

        @Override
        public boolean leftBumper() {
            return leftBumper;
        }

        @Override
        public boolean rightBumper() {
            return rightBumper;
        }

        @Override
        public double leftTrigger() {
            return leftTrigger;
        }

        @Override
        public double rightTrigger() {
            return rightTrigger;
        }

        @Override
        public boolean a() {
            return a;
        }

        @Override
        public boolean b() {
            return b;
        }

        @Override
        public boolean x() {
            return x;
        }

        @Override
        public boolean y() {
            return y;
        }

        @Override
        public boolean leftStickButton() {
            return leftStickButton;
        }

        @Override
        public boolean rightStickButton() {
            return rightStickButton;
        }
    }

    private static double[] getAxisValues(LogTable root) {
        LogTable joy0 = root.getSubtable("/DriverStation").getSubtable("Joystick0");
        double[] axes = joy0.get("AxisValues", new double[0]);
        if (axes.length < 6) {
            joy0 = root.getSubtable("DriverStation").getSubtable("Joystick0");
            axes = joy0.get("AxisValues", new double[0]);
        }
        if (axes.length >= 6) {
            return axes;
        }
        // Back-compat: some logs may have float[]
        float[] axesF = joy0.get("AxisValues", new float[0]);
        if (axesF.length >= 6) {
            return new double[] {
                    axesF[0], axesF[1], axesF[2], axesF[3], axesF[4], axesF[5]
            };
        }
        return new double[0];
    }

    private static boolean getDpadDown(LogTable root) {
        LogTable joy0 = root.getSubtable("/DriverStation").getSubtable("Joystick0");

        // Prefer explicit boolean if present.
        boolean dpadDown = joy0.get("DpadDown", false);

        // Fallback to POVs encoding (180 degrees).
        int[] povs = joy0.get("POVs", new int[] {-1});
        boolean povDown = povs.length > 0 && povs[0] == 180;

        // Back-compat: some logs may omit the leading slash.
        if (!dpadDown && !povDown) {
            LogTable joy0NoSlash = root.getSubtable("DriverStation").getSubtable("Joystick0");
            dpadDown = joy0NoSlash.get("DpadDown", false);
            povs = joy0NoSlash.get("POVs", new int[] {-1});
            povDown = povs.length > 0 && povs[0] == 180;
        }

        return dpadDown || povDown;
    }

    public static void main(String[] args) {
        String path = (args.length > 0) ? args[0] : RLOGReplay.promptForPath();

        RLOGReplay replay = new RLOGReplay(path);
        replay.start();

        DriveIOReplay io = new DriveIOReplay();
        Drive drive = new Drive(io);
        ReplayGamepad gamepad1 = new ReplayGamepad();
        ReplayGamepad gamepad2 = new ReplayGamepad();
        DriveControl driveControl = new DriveControl(drive, gamepad1, gamepad2);

        LogTable table = new LogTable(0);
        long tick = 0;
        while (replay.updateTable(table)) {
            if (tick == 0) {
                int shown = 0;
                System.out.println("First-tick key scan (DriverStation/Joystick):");
                for (String k : table.getAll(false).keySet()) {
                    if (k.contains("DriverStation") || k.contains("Joystick")) {
                        System.out.println("- " + k);
                        shown++;
                        if (shown >= 60) break;
                    }
                }
                if (shown == 0) {
                    System.out.println("- (none found)");
                }
            }

            // Match current TeleOp ordering (Robot.update() runs before DriveControl.update()).
            // That means powers computed from joystick at tick N are applied on tick N+1.
            drive.update();

            double[] axes = getAxisValues(table);
            if (axes.length < 6) {
                if (tick == 0) {
                    System.out.println("No /DriverStation/Joystick0/AxisValues found in this log; cannot replay drive.");
                }
                io.nextTick();
                tick++;
                continue;
            }

            // GamepadWrapper axis order:
            // 0=left_x, 1=left_y, 4=right_x
            gamepad1.setLeftStickX(axes[0]);
            gamepad1.setLeftStickY(axes[1]);
            gamepad1.setRightStickX(axes[4]);
            gamepad1.setDpadDown(getDpadDown(table));

            driveControl.update();

            if (tick < 20 || tick % 50 == 0) {
                System.out.printf(
                        "Tick %d: axes(lx=%.3f ly=%.3f rx=%.3f) slow=%s%n",
                        tick,
                        axes[0], axes[1], axes[4],
                        drive.useSlowMode
                );
            }

            io.nextTick();
            tick++;
        }

        replay.end();
    }
}
