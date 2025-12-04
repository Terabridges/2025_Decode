/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name="MotorTester", group="Test")
public class WheelTester extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    //TODO just make a drive object to declare motors

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            /// left back

            if (currentGamepad1.a && !previousGamepad1.a){
                leftBackDrive.setPower(1);
            }

            if (!currentGamepad1.a && previousGamepad1.a){
                leftBackDrive.setPower(0);
            }

            /// left front

            if (currentGamepad1.x && !previousGamepad1.x){
                leftFrontDrive.setPower(1);
            }

            if (!currentGamepad1.x && previousGamepad1.x){
                leftFrontDrive.setPower(0);
            }

            /// right front

            if (currentGamepad1.y && !previousGamepad1.y){
                rightFrontDrive.setPower(1);
            }

            if (!currentGamepad1.y && previousGamepad1.y){
                rightFrontDrive.setPower(0);
            }

            /// right back

            if (currentGamepad1.b && !previousGamepad1.b){
                rightBackDrive.setPower(1);
            }

            if (!currentGamepad1.b && previousGamepad1.b){
                rightBackDrive.setPower(0);
            }
        }
    }
}
