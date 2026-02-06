package org.firstinspires.ftc.teamcode.opmodes.tests.oldTests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.OLD.Transfer;

@Disabled
@Configurable
@TeleOp(name="TransferTester", group="Test")
public class transferTester extends LinearOpMode {

    private JoinedTelemetry joinedTelemetry;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Transfer transfer;
    public Intake intake;
    public Drive drive;
    public static double transferUp = 0.325;
    public static double transferDown = 0.385;
    public static double spinSpeed = 0.3;
    public static double spinnerPow = 1.0;
    public static double raiserLeftPos = 0.425;
    public static double raiserRightPos = 0.425;
    public static double raiserUpPos = 0.02;
    public static double raiserDownPos = 0.05;

    @Override
    public void runOpMode(){

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap);
        transfer.spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.a && !previousGamepad1.a){
                transfer.clutch.setPosition(transferDown);
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                transfer.clutch.setPosition(transferUp);
            }

            if(currentGamepad1.x && !previousGamepad1.x){
                intake.spinner.setPower(-spinnerPow);
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                intake.spinner.setPower(spinnerPow);
            }

            if(currentGamepad1.right_trigger>0){
                transfer.spindex.setPower(spinSpeed);
            } else if(currentGamepad1.left_trigger>0){
                transfer.spindex.setPower(-spinSpeed);
            } else {
                transfer.spindex.setPower(0);
            }

            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                intake.spinner.setPower(0);
            }

            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                intake.spinner.setPower(0);
            }

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                //intake.raiserLeft.setPosition(raiserLeftPos);
            }

            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                //intake.raiserRight.setPosition(raiserRightPos);
            }

            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
//                intake.raiserLeft.setPosition(0.425 + raiserUpPos);
//                intake.raiserRight.setPosition(0.425 - raiserUpPos);
            }

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
//                intake.raiserLeft.setPosition(0.425 - raiserDownPos);
//                intake.raiserRight.setPosition(0.425 + raiserDownPos);
            }

            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            drive.leftFront.setPower(leftFrontPower);
            drive.rightFront.setPower(rightFrontPower);
            drive.leftBack.setPower(leftBackPower);
            drive.rightBack.setPower(rightBackPower);

            joinedTelemetry.addData("Spindex Pos", transfer.spindex.getCurrentPosition());
            joinedTelemetry.addData("Spindex Pow", transfer.spindex.getPower());
//            joinedTelemetry.addData("Left Pos", intake.raiserLeft.getPosition());
//            joinedTelemetry.addData("Right Pos", intake.raiserRight.getPosition());
            joinedTelemetry.update();

            //left: 0.45, decreasing goes down
            //right: 0.4 increasing goes down

            //ALL THE WAY DOWN, left 0.36, right 0.49
            //equal at 0.425.
            //going up: left, 0.425+x right, 0.425-x
            //going down: left, 0.425-x right, 0.425+x
        }

    }


}
