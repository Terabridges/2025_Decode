package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive implements Subsystem{

    //---------------- Hardware ----------------
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
//    public GoBildaPinpointDriver pinpoint;

    //---------------- Software ----------------
    public boolean manualDrive = true;
    public boolean useSlowMode = false;
    double leftFrontPow = 0.0;
    double rightFrontPow = 0.0;
    double leftBackPow = 0.0;
    double rightBackPow = 0.0;

    public double FAST_MULT = 1.0;
    public double SLOW_MULT = 0.75;
    public double speed = FAST_MULT;
//    public boolean useFieldCentric = true;


    //---------------- Constructor ----------------
    public Drive(HardwareMap map) {
        leftBack = map.get(DcMotor.class, "left_back");
        rightBack = map.get(DcMotor.class, "right_back");
        leftFront = map.get(DcMotor.class, "left_front");
        rightFront = map.get(DcMotor.class, "right_front");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        pinpoint = map.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setOffsets(-1.527559, 5.70866, DistanceUnit.INCH);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        pinpoint.resetPosAndIMU();
    }

    //---------------- Methods ----------------
    public void setDrivePowers(double lf, double rf, double lb, double rb){
        leftFrontPow = lf;
        rightFrontPow = rf;
        leftBackPow = lb;
        rightBackPow = rb;
    }

    public void toggleSlowMode(){
        useSlowMode = !useSlowMode;
    }

//    public void toggleFieldCentric(){
//        useFieldCentric = !useFieldCentric;
//    }
//
//    public void resetPinpointIMU(){
//        pinpoint.recalibrateIMU();
//    }
//
//    public void driveFieldRelative(double forward, double right, double rotate) {
//        // First, convert direction being asked to drive to polar coordinates
//        double theta = Math.atan2(forward, right);
//        double r = Math.hypot(right, forward);
//
//        // Second, rotate angle by the angle the robot is pointing
//        theta = AngleUnit.normalizeRadians(theta -
//                pinpoint.getHeading(AngleUnit.RADIANS));
//
//        // Third, convert back to cartesian
//        double newForward = r * Math.sin(theta);
//        double newRight = r * Math.cos(theta);
//
//        // Finally, call the drive method with robot relative forward and right amounts
//        drive(newForward, newRight, rotate);
//    }
//
//    public void drive(double forward, double right, double rotate) {
//        // This calculates the power needed for each wheel based on the amount of forward,
//        // strafe right, and rotate
//        double frontLeftPower = forward + right + rotate;
//        double frontRightPower = forward - right - rotate;
//        double backRightPower = forward + right - rotate;
//        double backLeftPower = forward - right + rotate;
//
//        double maxPower = 1.0;
//
//        // This is needed to make sure we don't pass > 1.0 to any wheel
//        // It allows us to keep all of the motors in proportion to what they should
//        // be and not get clipped
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//
//        // We multiply by maxSpeed so that it can be set lower for outreaches
//        // When a young child is driving the robot, we may not want to allow full
//        // speed.
//        frontLeftPower /= maxPower;
//        frontRightPower /= maxPower;
//        backLeftPower /= maxPower;
//        backRightPower /= maxPower;
//
//        setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
//    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

//        pinpoint.update();

        speed = (useSlowMode ? SLOW_MULT : FAST_MULT);
        if(manualDrive){
            leftFrontPow*=speed;
            rightFrontPow*=speed;
            leftBackPow*=speed;
            rightBackPow*=speed;
            leftFront.setPower(leftFrontPow);
            rightFront.setPower(rightFrontPow);
            leftBack.setPower(leftBackPow);
            rightBack.setPower(rightBackPow);
        }
    }

}
