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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

//Added on Feb 19, 2020

//@Autonomous(name="OPENCV_Targeting_AutonomousV7", group ="Concept")
//@Disabled
public class Skystone_Autonomous_VisionTargetOPENCVversion7 extends BaseVisionOpMode {

    boolean problemChild = false;


    @Override
    public void runOpMode() {

        GetIMU();
        GetHardware();
        InitOpenCV();
        resetAngle();

        waitForStart();

        UnfoldRobotNoMovement();

        if (GetSkystonePosition() == 1) {
                webcam.closeCameraDevice();
                telemetry.addData("Skystone", "FarRight");
                telemetry.update();
                //EncoderDrive(DriveDirection.BACK_LEFT, 100);
                //EncoderDrive(DriveDirection.STRAFE_LEFT, 950);
                feeder_motor.setPower(1);
                encoderDrive(1, -12, 3);
                ResetEncoder();
                EncoderDrive(DriveDirection.STRAFE_LEFT, 950);
                encoderDrive(1, -32, 3.5);
                Clamp_Left.setPosition(.5);
                Clamp_Right.setPosition(.4);
                encoderDrive(1, 16, 3);
                rotate(89, 0.7);

            encoderDrive(1, -2, 3);
            //EncoderDrive(DriveDirection.BACKWARD, 100);
            top_motor.setPower(1);
            Lift(LiftDirection.DOWN);

            while(!problemChild || bottom_touch.getState())
            {
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



            Block_Pickup.setPosition(1f);
            sleep(600);
            Lift(LiftDirection.UP);
            sleep(25);
            Lift(LiftDirection.STOP);
            encoderDrive(DRIVE, 75.65, 8);



        } else if (GetSkystonePosition() == 2) {
                webcam.closeCameraDevice();
                telemetry.addData("Skystone", "Center");
                telemetry.update();
                feeder_motor.setPower(1);

                encoderDrive(1, -42, 3);
                Clamp_Left.setPosition(.5);
                Clamp_Right.setPosition(.4);
                encoderDrive(1, 18, 3);
                rotate(90, 0.7);

            top_motor.setPower(1);
            Lift(LiftDirection.DOWN);

            while(!problemChild || bottom_touch.getState())
            {
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
            EncoderDrive(DriveDirection.BACKWARD, 100);
            Block_Pickup.setPosition(1f);
            sleep(600);
            Lift(LiftDirection.UP);
            sleep(75);
            Lift(LiftDirection.STOP);
            encoderDrive(DRIVE, 76.5, 5);


        } else { // (GetSkystonePosition() == 3)
                webcam.closeCameraDevice();
                telemetry.addData("Skystone", "FarLeft");
                telemetry.update();
                feeder_motor.setPower(1);

                //top_motor.setPower(1);
            //strafes right because robot is backwards
                encoderDrive(1, -12, 3);
                ResetEncoder();
                EncoderDrive(DriveDirection.STRAFE_RIGHT, 900);
                encoderDrive(1, -32, 3.5);
                Clamp_Left.setPosition(.5);
                Clamp_Right.setPosition(.4);
                encoderDrive(1, 18, 3);
                rotate(90, 0.7);
                encoderDrive(1, 20, 2);

            top_motor.setPower(1);
            Lift(LiftDirection.DOWN);

            while(!problemChild || bottom_touch.getState())
            {
                if (!Top_Sensor_Rear.getState()) {
                    problemChild = true;
                    top_motor.setPower(0);
                }
                if (!bottom_touch.getState()) {
                    Lift(LiftDirection.STOP);
                }
            }

            EncoderDrive(DriveDirection.BACKWARD, 100);
            Block_Pickup.setPosition(1f);
            sleep(100);
            //Lift up a tiny bit to avoid friction
            Lift(LiftDirection.UP);
            sleep(50);
            Lift(LiftDirection.STOP);
            encoderDrive(DRIVE, 76.5, 5);




        }



//DOUGLAS'S CODE


            //Dropped block, moved to before robot goes under bridge
        //Block_Pickup.setPosition(1f);
        //sleep(900);

        //Lift(LiftDirection.UP);
        //sleep(150);
        //Lift(LiftDirection.STOP);

        resetAngle();
        rotate(85, 1);  //compensate for inertia


        //Reduced below drive distance from 10 to 8
        encoderDrive(DRIVE, 6, 3);

        Lift(LiftDirection.UP);
        Clamp_Left.setPosition(0.8f);
        Clamp_Right.setPosition(0.1f);
        sleep(425);
        Lift(LiftDirection.STOP);
        //Clamp_Left.setPosition(0.8f);
        //Clamp_Right.setPosition(0.1f);


        encoderDrive(DRIVE, 2, 1);


        while(Top_Sensor_Front.getState()){
            top_motor.setPower(-1);
        }
        top_motor.setPower(0);

        //feeder_motor.setPower(0);

        Lift(LiftDirection.DOWN);
        sleep(300);
        Lift(LiftDirection.STOP);

        resetAngle();
        rotate(-15, 1);
        //increased by 5in (from -10)
        //Incrased by 10 (from -25)
        encoderDrive(DRIVE, -14, 2);
        resetAngle();
        rotate(-80, 1);


        //to wall
        encoderDrive(DRIVE, 7, 2);

        Lift(LiftDirection.DOWN);
        sleep(200);
        Lift(LiftDirection.STOP);

        Clamp_Left.setPosition(.2);
        Clamp_Right.setPosition(.8);
        sleep(100);

        Block_Pickup.setPosition(.4f); //release Block
        encoderDrive(DRIVE, -12, 2);
        Lift(LiftDirection.DOWN);
        while(bottom_touch.getState())
        {
            idle();
        }
        Lift(LiftDirection.STOP);
        //Changed from -54 to -55
        encoderDrive(DRIVE, -55 , 8);

        feeder_motor.setPower(1);

        resetAngle();
        rotate(-30, .7);
        encoderDrive(DRIVE, -30, 2);

        encoderDrive(DRIVE, 30, 2);
        rotate(30, .7);
        encoderDrive(DRIVE, 45, 3);



        telemetry.update();


    }
}

