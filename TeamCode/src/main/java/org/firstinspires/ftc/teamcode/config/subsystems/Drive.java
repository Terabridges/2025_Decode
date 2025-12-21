package org.firstinspires.ftc.teamcode.config.subsystems;

import org.firstinspires.ftc.teamcode.config.subsystems.io.DriveIO;

public class Drive implements Subsystem{

    private final DriveIO io;
    private final DriveIO.Inputs inputs = new DriveIO.Inputs();

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
    public Drive(DriveIO io) {
        this.io = io;
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

        io.updateInputs(inputs);

        speed = (useSlowMode ? SLOW_MULT : FAST_MULT);
        if(manualDrive){
            leftFrontPow*=speed;
            rightFrontPow*=speed;
            leftBackPow*=speed;
            rightBackPow*=speed;
            io.setMotorPowers(leftFrontPow, rightFrontPow, leftBackPow, rightBackPow);
        }
    }

}
