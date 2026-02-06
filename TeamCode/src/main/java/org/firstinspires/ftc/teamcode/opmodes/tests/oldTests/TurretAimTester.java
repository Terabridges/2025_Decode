package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Shooter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "TurretAimTester", group = "Test")
public class TurretAimTester extends LinearOpMode {

    private enum TargetLock {
        NONE,
        BLUE_GOAL,
        RED_GOAL,
        OBELISK
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        Shooter shooter = robot.shooter;
        robot.drive.manualDrive = true;

        // Use a fresh follower and seed to blue far start pose.
        Follower follower = Constants.createFollower(hardwareMap);
        AutoPoses poses = new AutoPoses();
        follower.setStartingPose(poses.findStartPose(Alliance.BLUE, Range.LONG_RANGE));

        Gamepad gp = gamepad1;
        boolean prevStart = false;
        boolean prevA = false;
        boolean prevX = false;
        boolean prevY = false;
        TargetLock targetLock = TargetLock.NONE;

        telemetry.addLine("Turret Aim Tester: A=Toggle Blue goal, Y=Toggle Red goal, X=Toggle Obelisk");
        telemetry.addLine("START zeros pose to current");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update follower pose (odometry)
            follower.update();
            Pose pose = follower.getPose();

            if (gp.start && !prevStart) {
                follower.setStartingPose(pose != null ? pose : new Pose());
            }

            if (gp.a && !prevA) {
                targetLock = targetLock == TargetLock.BLUE_GOAL ? TargetLock.NONE : TargetLock.BLUE_GOAL;
            }
            if (gp.y && !prevY) {
                targetLock = targetLock == TargetLock.RED_GOAL ? TargetLock.NONE : TargetLock.RED_GOAL;
            }
            if (gp.x && !prevX) {
                targetLock = targetLock == TargetLock.OBELISK ? TargetLock.NONE : TargetLock.OBELISK;
            }

            if (pose != null) {
                switch (targetLock) {
                    case BLUE_GOAL:
                        // Blue goal at (0, 144)
                        shooter.aimTurretAtFieldPose(
                                pose.getX(),
                                pose.getY(),
                                pose.getHeading(),
                                0,
                                144);
                        break;
                    case RED_GOAL:
                        // Red goal at (144, 144)
                        shooter.aimTurretAtFieldPose(
                                pose.getX(),
                                pose.getY(),
                                pose.getHeading(),
                                144,
                                144);
                        break;
                    case OBELISK:
                        // Obelisk placeholder at midfield top edge
                        shooter.aimTurretAtFieldPose(
                                pose.getX(),
                                pose.getY(),
                                pose.getHeading(),
                                72,
                                144);
                        break;
                    case NONE:
                        shooter.useTurretPID = false;
                        shooter.useTurretLock = false;
                        break;
                }
            } else {
                shooter.useTurretPID = false;
                shooter.useTurretLock = false;
            }

            // Simple mecanum field-agnostic drive for repositioning during test.
            double y = -gp.left_stick_y; // forward/back
            double x = gp.left_stick_x;  // strafe
            double rx = gp.right_stick_x; // rotate
            double denom = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            double lf = (y + x + rx) / denom;
            double rf = (y - x - rx) / denom;
            double lb = (y - x + rx) / denom;
            double rb = (y + x - rx) / denom;
            robot.drive.setDrivePowers(lf, rf, lb, rb);

            shooter.update();
            robot.drive.update(); // keep drive motors idle/update states

            Pose p = pose;
            telemetry.addData("Turret Target", shooter.turretTargetDeg);
            telemetry.addData("Turret Pos", shooter.getTurretPos());
            if (p != null) {
                telemetry.addData("Pose", "%.1f, %.1f, %.1f deg",
                        p.getX(),
                        p.getY(),
                        Math.toDegrees(p.getHeading()));
            }
            telemetry.addData("Target Lock", targetLock);
            telemetry.update();

            prevStart = gp.start;
            prevA = gp.a;
            prevX = gp.x;
            prevY = gp.y;
        }
    }
}
