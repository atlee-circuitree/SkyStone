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

//import static org.firstinspires.ftc.teamcode.BaseOpMode.DriveDirection.STRAFE_RIGHT;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

//@Autonomous(name = "Skystone_Foundation_RedOnly", group = "Linear Opmode")
public class Skystone_Foundation_Red_Only extends BaseAutoOpMode {



    double globalAngle, power = 1, correction;

    int startingSide = -1;  //Set to 1 for blue and -1 for Red
    boolean problemChild = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        //Assigns hardware devices names and values

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


//unfolds here
        encoderDrive(DRIVE, 5, 3);
        UnfoldRobot(); //Created a new method: UnfoldRobotFoundation; 2/15/20 9:45
        EncoderDrive(DriveDirection.STRAFE_RIGHT, 1337);

        encoderDrive(DRIVE, 26, 3);

//The robot lowers the foundation clamps
        Clamp_Left.setPosition(0.9f);
        Clamp_Right.setPosition(0f);
        sleep(750);

        //rotate(-15, 1);
        //encoderDrive(DRIVE, -18, 4);
       //resetAngle();

        //copied foundation move from skystone code
      //  EncoderDrive(DriveDirection.STRAFE_LEFT, 500);

        curvedRotate(-90, -.10, -1);


//        EncoderDrive(DriveDirection.TURN_RIGHT, 800);


        //EncoderDrive(DriveDirection.STRAFE_LEFT, 5000); //added 2/15/20- changing turn to strafe
        encoderDrive(DRIVE, 6, 2);
        Clamp_Left.setPosition(0);
        Clamp_Left.setPosition(1);
        sleep(750);

        top_motor.setPower(1);
        Lift(LiftDirection.DOWN);
        while(!problemChild || bottom_touch.getState()) {
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

        encoderDrive(DRIVE, -50, 5);

        //Dylan Settersten's contribution, if you are confused or need help, ask Alex English or Phil Issacs for my contact info
        //replace 1 with the number of stones
          int SkyNumber = 1;
        if (SkyNumber == 0){

            encoderDrive(DRIVE, 48, 4);

        } else {
            for (int BlockCounter = 1; BlockCounter <= SkyNumber; BlockCounter++) {


                encoderDrive(DRIVE, 10, 4);
                EncoderDrive(DriveDirection.STRAFE_RIGHT, 1000); //if position is negative change to STRAFE_LEFT

                feeder_motor.setPower(-1); //getting that block
                sleep(250);
                feeder_motor.setPower(0);

                //grabbing function

                encoderDrive(DRIVE, -10, 4);
                EncoderDrive(DriveDirection.STRAFE_LEFT, 1000); //if first strafe function is STRAFE_RIGHT, change to STRAFE_LEFT

                //placing function
                //reset height

            }
        }

        /*Ignore for now
        for (
         */

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}

