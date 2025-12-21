package org.firstinspires.ftc.teamcode.config.control;

import org.firstinspires.ftc.teamcode.config.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class DriveControl implements Control {

    //---------------- Software ----------------
    Drive drive;
    GamepadView gp1;
    GamepadView gp2;
    EdgeDetector slowModeRE = new EdgeDetector( () -> drive.toggleSlowMode());
//    EdgeDetector toggleFieldCentric = new EdgeDetector(()-> drive.toggleFieldCentric());
//    EdgeDetector resetIMU = new EdgeDetector(()-> drive.resetPinpointIMU());


    //---------------- Constructor ----------------
    public DriveControl(Drive drive, GamepadView gp1, GamepadView gp2){
        this.drive = drive;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------

//    if (drive.useFieldCentric){
//        drive.driveFieldRelative(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x);
//    } else {

    @Override
    public void update(){

        slowModeRE.update(gp1.dpadDown());
//        resetIMU.update(gp1.right_stick_button);
//        toggleFieldCentric.update(gp1.left_stick_button);

        if(drive.manualDrive){
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gp1.leftStickY();  // Note: pushing stick forward gives negative value
            double lateral = gp1.leftStickX();
            double yaw = gp1.rightStickX();
            double[] powers = MecanumDriveMath.computeWheelPowers(axial, lateral, yaw);
            drive.setDrivePowers(powers[0], powers[1], powers[2], powers[3]);
        }
    }

    @Override
    public void addTelemetry(TelemetrySink telemetry){
        telemetry.addData("Slow Mode?", drive.useSlowMode);
//        telemetry.addData("Use Field Centric?", drive.useFieldCentric);
//        telemetry.addData("Heading", drive.pinpoint.getHeading(AngleUnit.RADIANS));
    }
}
