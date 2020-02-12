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

@Autonomous(name="OPENCV_Targeting_Autonomous", group ="Concept")
//@Disabled
public class Skystone_Autonomous_VisionTargetOPENCV extends BaseVisionOpMode {

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
                telemetry.addData("Skystone", "FarRight");
                telemetry.update();
                EncoderDrive(DriveDirection.STRAFE_LEFT, 950);
                feeder_motor.setPower(1);
                encoderDrive(1, -50, 3);
                encoderDrive(1, 30, 3);
                rotate(90, 0.7);


        } else if (GetSkystonePosition() == 2) {
                telemetry.addData("Skystone", "Center");
                telemetry.update();
                feeder_motor.setPower(1);
                encoderDrive(1, -50, 3);
                encoderDrive(1, 30, 3);
                rotate(90, 0.7);
                //EncoderDrive(DriveDirection.FORWARD, 200);


        } else if (GetSkystonePosition() == 3) {
                telemetry.addData("Skystone", "FarLeft");
                telemetry.update();
                feeder_motor.setPower(1);
                EncoderDrive(DriveDirection.STRAFE_RIGHT, 900);
                encoderDrive(1, -50, 3);
                encoderDrive(1, 30, 3);
                rotate(90, 0.7);
                encoderDrive(1, 24, 2);

        } else {
                telemetry.addData("Skystone", "Gosh Darn It");
                telemetry.update();
        }




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


        //DOUGLASSES CODE



        encoderDrive(DRIVE, 76.5, 5);
        encoderDrive(DRIVE, -3,1);

        Block_Pickup.setPosition(1f);
        sleep(900);

        Lift(LiftDirection.UP);
        sleep(150);
        Lift(LiftDirection.STOP);

        resetAngle();
        rotate(79, 1);


        encoderDrive(DRIVE, 10, 1);

        Lift(LiftDirection.UP);
        Clamp_Left.setPosition(0.7f);
        Clamp_Right.setPosition(0.3f);
        sleep(400);
        Lift(LiftDirection.STOP);
        sleep(200);

        encoderDrive(DRIVE, 2, 1);


        while(Top_Sensor_Front.getState()){
            top_motor.setPower(-1);
        }
        top_motor.setPower(0);

        feeder_motor.setPower(0);

        Lift(LiftDirection.DOWN);
        sleep(700);
        Lift(LiftDirection.STOP);

        Block_Pickup.setPosition(.4f);
        sleep(800);
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


        EncoderDrive(DriveDirection.BACK_LEFT, 3500);

        encoderDrive(DRIVE, 10, 2);

        Lift(LiftDirection.UP);
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        sleep(600);
        Lift(LiftDirection.STOP);
        encoderDrive(DRIVE, 12, 2);


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

        encoderDrive(DRIVE, -64 , 8);

        feeder_motor.setPower(1);

        resetAngle();
        rotate(-30, 1);
        encoderDrive(DRIVE, -20, 2 );

        encoderDrive(DRIVE, 20, 2);
        rotate(30, 1);
        encoderDrive(DRIVE, 34, 3);



        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
}

