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

@Autonomous(name = "Red_BridgeSide", group = "Linear Opmode")
public class Skystone_Foundation_Red_Bridge_Side extends BaseAutoOpMode {



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
        UnfoldRobot();
        resetAngle();
        EncoderDrive(DriveDirection.STRAFE_RIGHT, 1033);

        encoderDrive(DRIVE, 25.5, 3);
        //Increased to 4 from 3
        encoderDrive(DRIVE, 4, 3);

        Clamp_Left.setPosition(0.9f);
        Clamp_Right.setPosition(0f);
        sleep(750);

        resetAngle();
        rotate(-15, 1);
        //increased by 5in (from -10)
        encoderDrive(DRIVE, -15, 2);
        resetAngle();
        rotate(-65, 1);
        encoderDrive(DRIVE, 13.5, 2);
        Clamp_Left.setPosition(.2);
        Clamp_Right.setPosition(.8);
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
        encoderDrive(DRIVE, -30, 5);

        //replace 1 with the number of stones
        int SkyNumber = 4;
        if (SkyNumber == 0) {

            encoderDrive(DRIVE, -6, 2);

        } else {
            for (int BlockCounter = 1; BlockCounter <= SkyNumber; BlockCounter++) {
                telemetry.addData("For Loop Counter", BlockCounter);
                telemetry.update();
                //Return tail back and lowest crane position
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
                feeder_motor.setPower(1); //getting that block
                sleep(250);

                encoderDrive(DRIVE, -18, 2);


                encoderDrive(1, -2, 3);
                //EncoderDrive(DriveDirection.BACKWARD, 100);
                Block_Pickup.setPosition(1f);
                sleep(300);
                feeder_motor.setPower(0);

                Lift(LiftDirection.UP);
                sleep(100); //changed from 10ms to 100ms 8:32
                Lift(LiftDirection.STOP);

                encoderDrive(DRIVE, 20, 3);

                Lift(LiftDirection.UP);
                sleep(425);
                Lift(LiftDirection.STOP);
                //Clamp_Left.setPosition(0.8f);
                //Clamp_Right.setPosition(0.1f);


                while(Top_Sensor_Front.getState()){
                    top_motor.setPower(-1);
                }
                top_motor.setPower(0);

                Lift(LiftDirection.DOWN);
                sleep(400);
                Lift(LiftDirection.STOP);

                Block_Pickup.setPosition(.4f);   //release Block
                sleep(300);

                //Lift(LiftDirection.DOWN);
                //sleep(100);
                //Lift(LiftDirection.STOP);




                Lift(LiftDirection.UP);
                sleep(800);
                Lift(LiftDirection.STOP);
                //Clamp_Left.setPosition(0.8f);
                //Clamp_Right.setPosition(0.1f);


                while(Top_Sensor_Front.getState()){
                    top_motor.setPower(1);
                }
                top_motor.setPower(0);


            }
        }


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}
