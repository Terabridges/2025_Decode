package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.utility.VirtualFence;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleop extends LinearOpMode {

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    public Gamepad currentGamepad2;
    public Gamepad previousGamepad2;

    double W = 3.6 / 2.0 - 0.10;
    double H = 1.8 / 2.0 - 0.10;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public GoBildaPinpointDriver pinpoint;

    VirtualFence fence;

    @Override
    public void runOpMode() throws InterruptedException {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(1, -3.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        fence = new VirtualFence();

    // Walls: n · x ≤ c. Outward normals point OUT of the allowed region.
        fence.addWall(VirtualFence.Wall.fromPointNormal(new VirtualFence.Vec2( W, 0.0), new VirtualFence.Vec2(+1, 0))); // +X wall
        fence.addWall(VirtualFence.Wall.fromPointNormal(new VirtualFence.Vec2(-W, 0.0), new VirtualFence.Vec2(-1, 0))); // -X wall
        fence.addWall(VirtualFence.Wall.fromPointNormal(new VirtualFence.Vec2(0.0,  H), new VirtualFence.Vec2(0, +1))); // +Y wall
        fence.addWall(VirtualFence.Wall.fromPointNormal(new VirtualFence.Vec2(0.0, -H), new VirtualFence.Vec2(0, -1))); // -Y wall
    // Example obstacle: a keep-out bubble around a field prop at (x0, y0) with inflated radius
        fence.addBubble(new VirtualFence.Bubble(new VirtualFence.Vec2(0.90, 0.30), 0.25)); // meters

        pinpoint.resetPosAndIMU();
        waitForStart();

        while (opModeIsActive()){
            pinpoint.update();

            double x = pinpoint.getEncoderX();      // meters (field frame)
            double y = pinpoint.getEncoderY();
            double h = pinpoint.getHeading(AngleUnit.RADIANS); // radians

            double driveX = -gamepad1.left_stick_y; // forward/back
            double driveY = gamepad1.left_stick_x;  // strafe
            double speed = 1.6;

            double vxDes =  speed * ( driveX * Math.cos(h) - driveY * Math.sin(h));
            double vyDes =  speed * ( driveX * Math.sin(h) + driveY * Math.cos(h));
            VirtualFence.Vec2 vDes = new VirtualFence.Vec2(vxDes, vyDes);
            VirtualFence.Pose2 pose = new VirtualFence.Pose2(x, y, h);

            // 1) Filter by virtual walls + bubbles
            VirtualFence.Vec2 vSafe = fence.filterVelocity(vDes, pose);

            // 2) Optionally “ride” along the nearest wall when close
            vSafe = fence.rideAlongIfClose(vSafe, pose);

            // 3) Auto-align heading near walls
            double omegaCmd = fence.alignmentOmega(pose);

            // Convert (vx, vy, ω) back to your drive’s power commands (mecanum/field-centric drive)
            setFieldCentricVelocity(vSafe.x, vSafe.y, omegaCmd, pinpoint.getHeading(AngleUnit.RADIANS));

        }
    }

    public void setFieldCentricVelocity(double vx, double vy, double omega, double heading)
    {
        // Convert field-centric (vx, vy) to robot-centric
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double vF = vx * cos - vy * sin; // forward
        double vS = vx * sin + vy * cos; // strafe

        // Standard mecanum kinematics (robot frame velocities → wheel powers)
        double fl = vF + vS + omega;
        double fr = vF - vS - omega;
        double bl = vF - vS + omega;
        double br = vF + vS - omega;

        // Normalize so max magnitude ≤ 1
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);
    }
}
