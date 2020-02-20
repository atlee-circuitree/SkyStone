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

    Orientation lastAnglesPID = new Orientation();
    double                  globalAnglePID, power = .30, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        GetIMU();
        GetHardware();

        // get a reference to REV Touch sensor.
        //touch = hardwareMap.touchSensor.get("touch_sensor");


        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

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

        while (opModeIsActive())
        {
            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAnglesPID.firstAngle);
            telemetry.addData("2 global heading", globalAnglePID);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.update();

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
                if (touched || aButton) rotatePID(-90, power);

                // turn 90 degrees left.
                if (bButton) rotatePID(90, power);
            }
        }

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    void resetAnglePID()
    {
        lastAnglesPID = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAnglePID = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAnglePID()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAnglePID = angles.firstAngle - lastAnglesPID.firstAngle;

        if (deltaAnglePID < -180)
            deltaAnglePID += 360;
        else if (deltaAnglePID > 180)
            deltaAnglePID -= 360;

        globalAnglePID += deltaAnglePID;

        lastAnglesPID = angles;

        return globalAnglePID;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotatePID(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAnglePID();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAnglePID() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAnglePID() == 0)
            {
                front_left.setPower(power);
                rear_left.setPower(power);
                front_right.setPower(-power);
                rear_right.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAnglePID()); // power will be - on right turn.
                front_left.setPower(-power);
                rear_left.setPower(-power);
                front_right.setPower(power);
                rear_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAnglePID()); // power will be + on left turn.
                front_left.setPower(-power);
                rear_left.setPower(-power);
                front_right.setPower(power);
                rear_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);

        rotation = getAnglePID();




        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAnglePID();
    }
}
