/* Copyright (c) 2019 FIRST. All rights reserved.
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


/**
 OpenCV-based
 */

//Added on Feb 20, 2020 11:22 AM
//Last Edited on Feb 20, 2020 11:22 AM - Larson
//Edited on Feb 20, 2020 7:40 PM - Berg - reducing the number of small differences between positions 1, 2 and 3

@Autonomous(name = "TWO_STONES_RED", group = "Concept")
public class Skystone_Autonomous_VisionTargetOPENCVNOPLATFORMRED extends BaseVisionOpMode {

    boolean problemChild = false;


    @Override
    public void runOpMode() {

        GetIMU();
        GetHardware();
        InitOpenCV();
        resetAngle();

        waitForStart();

        UnfoldRobotNoMovement();

        int SkystonePosition = GetSkystonePosition();
        webcam.closeCameraDevice();
        feeder_motor.setPower(1);

        //Drive into block
        if(SkystonePosition == 1)
        {
            telemetry.addData("Skystone", "FarRight");
            telemetry.update();
            ResetEncoder();
            encoderDrive(1, -12, 3);
            ResetEncoder();
            EncoderDrive(DriveDirection.STRAFE_LEFT, 950);
            ResetEncoder();
            encoderDrive(1, -32, 3.5);
        }
        else if(SkystonePosition == 2)
        {
            telemetry.addData("Skystone", "Center");
            telemetry.update();
            ResetEncoder();
            encoderDrive(1, -44, 3);
        }
        else
        {
            telemetry.addData("Skystone", "FarLeft");
            telemetry.update();

            //top_motor.setPower(1);
            //strafes right because robot is backwards
            encoderDrive(1, -12, 3);
            ResetEncoder();
            //increased to 1000 (from 950)
            EncoderDrive(DriveDirection.STRAFE_RIGHT, 1000);
            ResetEncoder();
            encoderDrive(1, -32, 3.5);
        }

        //position clamps
        Clamp_Left.setPosition(.5);
        Clamp_Right.setPosition(.4);

        //Drive back into lane
        ResetEncoder();
        encoderDrive(1, 16, 3);

        //Move back to common position
        if(SkystonePosition == 1)
        {
            ResetEncoder();
            EncoderDrive(DriveDirection.STRAFE_RIGHT, 950); //get back to starting position

        }
        else if(SkystonePosition == 2)
        {
            //no change needed
        }
        else
        {
            ResetEncoder();
            EncoderDrive(DriveDirection.STRAFE_LEFT, 1000); //get back to starting position
        }

        ResetEncoder();
        //Turn towards Foundation side
        rotate(-90, 0.7);

        //Position Crane and Lift before going under bridge
        top_motor.setPower(1);
        Lift(LiftDirection.DOWN);

        while (!problemChild || bottom_touch.getState()) {
            if (Top_Sensor_Rear.getState()) {
            } else {
                problemChild = true;
                top_motor.setPower(0);
            }
            if (bottom_touch.getState()) {
            } else {
                Lift(LiftDirection.STOP);

            }
        }

        ResetEncoder();
        encoderDrive(DRIVE, -48, 8);
        feeder_motor.setPower(-1);
        sleep(300);
        feeder_motor.setPower(1);
        rotate(175, 1);

        //Drive back to 2nd skystone - 8 inches between each stone
        ResetEncoder();
        if(SkystonePosition == 1)
        {
            encoderDrive(DRIVE, -56, 5);
        }
        else if(SkystonePosition == 2)
        {
            encoderDrive(DRIVE, -72, 8);
        }
        else
        {
            encoderDrive(DRIVE, -72, 8);
        }
        ResetEncoder();
        EncoderDrive(DriveDirection.STRAFE_LEFT, 1900);
        feeder_motor.setPower(1);
        ResetEncoder();
        //Get stone into bay
        encoderDrive(DRIVE, -10, 2);
        feeder_motor.setPower(0);
        ResetEncoder();
        //Drive back into lane
        EncoderDrive(DriveDirection.STRAFE_RIGHT, 1900);
        feeder_motor.setPower(1);
        rotate(3, 1);

        //Drive back to foundation side
        ResetEncoder();
        if(SkystonePosition == 1)
        {
            encoderDrive(DRIVE, 56, 8);
        }
        else if(SkystonePosition == 2)
        {
            encoderDrive(DRIVE, 72, 8);
        }
        else
        {
            encoderDrive(DRIVE, 72, 8);
        }
        feeder_motor.setPower(-1);
        sleep(500);

        encoderDrive(1, -12, 2);
        telemetry.update();
    }
}

