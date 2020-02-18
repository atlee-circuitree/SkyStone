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

@Autonomous(name="OPENCV_Targeting_AutonomousV6", group ="Concept")
//@Disabled
public class Skystone_Autonomous_VisionTargetOPENCVversion6 extends BaseVisionOpMode {

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
                encoderDrive(1, 18, 3);
                rotate(90, 0.7);

            encoderDrive(1, -1, 8);
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
            sleep(50);
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


        } else if (GetSkystonePosition() == 3) {
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
            //Lift up a tiny bit to avoid friction
            Lift(LiftDirection.UP);
            sleep(50);
            Lift(LiftDirection.STOP);
            encoderDrive(DRIVE, 76.5, 5);




        } else {
                telemetry.addData("Skystone", "Gosh Darn It");
                telemetry.update();
        }



//DOUGLAS'S CODE


            //Dropped block, moved to before robot goes under bridge
        //Block_Pickup.setPosition(1f);
        //sleep(900);

        Lift(LiftDirection.UP);
        sleep(175);
        Lift(LiftDirection.STOP);

        resetAngle();
        rotate(87, 1);  //compensate for inertia


        //Reduced below drive distance from 10.5 to 9
        encoderDrive(DRIVE, 10, 3);

        Lift(LiftDirection.UP);
        Clamp_Left.setPosition(0.8f);
        Clamp_Right.setPosition(0.1f);
        sleep(400);
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
        sleep(800);
        Lift(LiftDirection.STOP);

        EncoderDrive(DriveDirection.STRAFE_LEFT, 100);
        EncoderDrive(DriveDirection.STRAFE_RIGHT, 100);
        encoderDrive(DRIVE, .5, 1);
        //curvedRotate(-45, .5, 1);


        //commented out to try 45 backup
        //resetAngle();
        //rotate(-10, 1);

        //encoderDrive(DRIVE, -16, 3);

      /*front_left.setPower(-1);
      front_right.setPower(1);
      rear_left.setPower(1);
      rear_right.setPower(-1);
      //EncoderDrive(DriveDirection.STRAFE_LEFT, 5000);
      sleep(2000);


       */
        //resetAngle();
        //rotate(-70,1);
       //Clamp_Left.setPosition(0.85f);
        //Clamp_Right.setPosition(0.1f);
        Block_Pickup.setPosition(.4f); //release Block
       // encoderDrive(1, -25, 3.0);
        resetAngle();
        rotate(-10, 1);
        encoderDrive(1, -16, 8);
        resetAngle();
        rotate(-80, 1);

        //sleep(100);

        //rotate(-80, 1);
        //rotateNoSlowDown(-70, 1);

        //encoderDrive(DRIVE, 10, 2);



        Lift(LiftDirection.UP);
        Block_Pickup.setPosition(1f);
        sleep(600);
        Lift(LiftDirection.STOP);
        Clamp_Left.setPosition(.5);
        Clamp_Right.setPosition(.4);
        Block_Pickup.setPosition(1f);

        //Pushing the foundation towards the wall
        //Larson and Will Stevens - Reduced below drive distance from 19 to 11
        encoderDrive(DRIVE, 11, 2);
        //EncoderDrive(DriveDirection.FORWARD, 1000);


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

       /* while(Top_Sensor_Rear.getState()){
            top_motor.setPower(1);
        }
        top_motor.setPower(0);

        while (bottom_touch.getState()){
            Lift(LiftDirection.DOWN);
        }
        Lift(LiftDirection.STOP);
        */

       //encoderDrive(DRIVE, -30, 8);

        encoderDrive(DRIVE, -74 , 8);

        feeder_motor.setPower(1);

        resetAngle();
        rotate(-30, 1);
        encoderDrive(DRIVE, -25, 2);

        encoderDrive(DRIVE, 25, 2);
        rotate(-150, 1);
        encoderDrive(DRIVE, -45, 3);

        //why does it rotate perfectly in line with... running into the bridge
        //rotate(-120, 1);
        //encoderDrive(DRIVE, 45, 3);



        telemetry.addData("Simon", "It failed");
        telemetry.update();


    }
}

