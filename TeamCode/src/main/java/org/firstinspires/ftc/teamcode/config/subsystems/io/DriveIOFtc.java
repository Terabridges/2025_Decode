package org.firstinspires.ftc.teamcode.config.subsystems.io;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class DriveIOFtc implements DriveIO {

    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;

    public DriveIOFtc(HardwareMap hardwareMap) {
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Keep existing behavior: open-loop power control.
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        // Not currently used by Drive logic, but captured so replay/sim can hook in later.
        inputs.leftFrontPosTicks = leftFront.getCurrentPosition();
        inputs.rightFrontPosTicks = rightFront.getCurrentPosition();
        inputs.leftBackPosTicks = leftBack.getCurrentPosition();
        inputs.rightBackPosTicks = rightBack.getCurrentPosition();

        inputs.leftFrontVelTicksPerSec = leftFront.getVelocity();
        inputs.rightFrontVelTicksPerSec = rightFront.getVelocity();
        inputs.leftBackVelTicksPerSec = leftBack.getVelocity();
        inputs.rightBackVelTicksPerSec = rightBack.getVelocity();
    }

    @Override
    public void setMotorPowers(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }
}
