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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "DriveAvoidPID", group = "Linear Opmode")
public class Skystone_Autonomous_DriveAvoidPID extends BaseAutoOpMode {


    double                  globalAnglePID, power = .30, rotatePower = 1, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        GetIMU();
        GetHardware();

        ResetEncoder();

        // get a reference to REV Touch sensor.
        //touch = hardwareMap.touchSensor.get("touch_sensor");


        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(0,0,0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // drive until end of period.
        int loops = 0;

        while (opModeIsActive())
        {
            loops++;
            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAnglesPID.firstAngle);
            telemetry.addData("2 global heading", globalAnglePID);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.addData("Loops", loops);

            // set power levels.
            front_left.setPower(power - correction);
            rear_left.setPower(power - correction);
            front_right.setPower(power + correction);
            rear_right.setPower(power + correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;
            touched = false;  //hardcoded - no button on bot currently - BERG

            if (touched || aButton || bButton)
            {
                telemetry.addData("Button Pushed", "Backing up");
                telemetry.update();
                // backup.
                front_left.setPower(-power);
                rear_left.setPower(-power);
                front_right.setPower(-power);
                rear_right.setPower(-power);

                sleep(500);

                // stop.
                front_left.setPower(0);
                rear_left.setPower(0);
                front_right.setPower(0);
                rear_right.setPower(0);


                // turn 90 degrees right.
                if (touched || aButton)
                {
                    telemetry.addData("Button Pushed", "Rotating right");
                    telemetry.update();
                    rotatePID(-90, rotatePower, pidRotate);
                }

                // turn 90 degrees left.
                if (bButton)
                {
                    telemetry.addData("Button Pushed", "Rotating left");
                    telemetry.update();
                    rotatePID(90, rotatePower, pidRotate);
                }
            }
            telemetry.update();

        }

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);
    }






}
