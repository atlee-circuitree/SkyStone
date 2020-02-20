/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 * ...Does anybody even read these things? I mean like, this hasn't been updated since I started coding...
 */

//@TeleOp(name = "TestMecanumOmnidrive", group = "Linear Opmode")
public class Skystone_Test_MecanumOmnidrive extends BaseOpMode {

    @Override
    public void runOpMode() {


        //float Power = 0;
        int CranePos = 1;

        GetHardware();

        waitForStart();


        while (opModeIsActive()) {

            if(gamepad1.left_stick_x != 0){

                    float JoystickX = gamepad1.left_stick_x;
                    float JoystickY = gamepad1.left_stick_y * -1;

                  //Power = 0;

                  front_left.setPower(-JoystickX + JoystickY);
                  rear_left.setPower(JoystickX + JoystickY);
                  front_right.setPower(JoystickX + JoystickY);
                  rear_right.setPower(-JoystickX + JoystickY);



            }
            if(gamepad1.dpad_left) {

                if (CranePos == 1) {
                    top_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    while (Top_Sensor_Rear.getState()) {
                        top_motor.setPower(0.5);
                        telemetry.addData("Encoder Value", top_motor.getCurrentPosition());
                        telemetry.addData("CranePosition", CranePos);
                        telemetry.update();
                    }
                    top_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    CranePos = 0;
                } else if (CranePos == 2) {
                    top_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    while (top_motor.getCurrentPosition() <= 10) {
                        top_motor.setPower(0.5);
                        telemetry.addData("Encoder Value", top_motor.getCurrentPosition());
                        telemetry.addData("CranePosition", CranePos);
                        telemetry.update();
                    }
                    top_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    CranePos = 1;
                }
            }
            else if(gamepad1.dpad_right) {

                if (CranePos == 1) {
                    top_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    while (Top_Sensor_Front.getState()) {
                        top_motor.setPower(-0.5);
                        telemetry.addData("Encoder Value", top_motor.getCurrentPosition());
                        telemetry.addData("CranePosition", CranePos);
                        telemetry.update();
                    }
                    top_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    CranePos = 2;
                } else if (CranePos == 0) {
                    top_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    while (top_motor.getCurrentPosition() >= -10) {
                        top_motor.setPower(-0.5);
                        telemetry.addData("Encoder Value", top_motor.getCurrentPosition());
                        telemetry.addData("CranePosition", CranePos);
                        telemetry.update();
                    }
                    top_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    CranePos = 1;
                }
            }

        }

    }
}