package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "TurretAimTester", group = "Test")
public class TurretAimTester extends LinearOpMode {

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
        boolean forceBlue = false;

        telemetry.addLine("Turret Aim Tester: A=Blue goal, Y=Red goal, X=Obelisk");
        telemetry.addLine("B toggles Force Blue; START zeros pose to current");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update follower pose (odometry)
            follower.update();

            if (gp.start && !prevStart) {
                Pose current = follower.getPose();
                follower.setStartingPose(current != null ? current : new Pose());
            }
            if (gp.b && !prevStart) {
                forceBlue = !forceBlue;
            }

            if (forceBlue || gp.a) {
                // Blue goal at (0, 144)
                shooter.aimTurretAtFieldPose(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getPose().getHeading(),
                        0,
                        144);
            } else if (gp.y) {
                // Red goal at (144, 144)
                shooter.aimTurretAtFieldPose(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getPose().getHeading(),
                        144,
                        144);
            } else if (gp.x) {
                // Obelisk placeholder at midfield top edge
                shooter.aimTurretAtFieldPose(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getPose().getHeading(),
                        72,
                        144);
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

            Pose p = follower.getPose();
            telemetry.addData("Turret Target", shooter.turretTargetDeg);
            telemetry.addData("Turret Pos", shooter.getTurretPos());
            if (p != null) {
                telemetry.addData("Pose", "%.1f, %.1f, %.1f deg",
                        p.getX(),
                        p.getY(),
                        Math.toDegrees(p.getHeading()));
            }
            telemetry.addData("Force Blue", forceBlue);
            telemetry.update();

            prevStart = gp.start;
        }
    }
}
