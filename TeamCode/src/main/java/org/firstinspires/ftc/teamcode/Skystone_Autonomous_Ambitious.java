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

import android.telephony.euicc.DownloadableSubscription;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

//@Autonomous(name = "Skystone_Autonomous_Ambitious", group = "Linear Opmode")
public class Skystone_Autonomous_Ambitious extends BaseAutoOpMode {



    double globalAngle, power = 1, correction;

    int startingSide = -1;  //Set to 1 for blue and -1 for Red



    @Override
    public void runOpMode() {

        /*
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

        feeder_motor.setPower(1);
        Block_Pickup.setPosition(.4f);
        EncoderDriveExactDistance(DRIVE, -50, 3);

        EncoderDriveExactDistance(DRIVE, 26, 3);

        while(Top_Sensor_Rear.getState()){
            top_motor.setPower(1);
        }
        top_motor.setPower(0);

        while (bottom_touch.getState()){
            Lift(LiftDirection.UP);
        }
        Lift(LiftDirection.STOP);

        EncoderDriveExactDistance(DRIVE, -3, 1);

        Block_Pickup.setPosition(1f);
        sleep(1000);

        resetAngle();
        rotate(79, .70);

        EncoderDriveExactDistance(DRIVE, 76, 3);

        rotate(79, .70);

        EncoderDriveExactDistance(DRIVE, 8, 2);

        Clamp_Left.setPosition(0.8f);
        Clamp_Right.setPosition(0f);
        sleep(1000);

        Lift(LiftDirection.DOWN);
        sleep(500);
        Lift(LiftDirection.STOP);

        while(Top_Sensor_Front.getState()){
            top_motor.setPower(-1);
        }
        top_motor.setPower(0);

        front_left.setPower(-(1));
        rear_left.setPower((0.2));
        front_right.setPower((0.2));
        rear_right.setPower(-(1));
        sleep(1500);


        Lift(LiftDirection.UP);
        sleep(600);


        resetAngle();
        rotate(80, .75);

        Block_Pickup.setPosition(.4f);
        EncoderDriveExactDistance(DRIVE, 24, 2);




        telemetry.addData("Path", "Complete");
        telemetry.update();


 */

    }
}
