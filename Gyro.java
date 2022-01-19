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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "Sensor: BNO055 IMU", group = "Sensor")

public class Gyro extends LinearOpMode
    {
        
    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_left;
    DcMotor back_right;
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double wanted_direction = 0;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.mode = BNO055IMU.SensorMode.IMU;
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.loggingEnabled = false;
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        
        front_left = hardwareMap.get(DcMotor.class,"front_left");
        front_right = hardwareMap.get(DcMotor.class,"front_right");
        back_left = hardwareMap.get(DcMotor.class,"back_left");
        back_right = hardwareMap.get(DcMotor.class,"back_right");
        
        back_right.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        
        
        
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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

    
        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
             correction = checkDirection();
             telemetry.addData("1 imu heading", lastAngles.firstAngle);
             telemetry.addData("wanted direction",wanted_direction);
             telemetry.addData("2 global heading", globalAngle);
             telemetry.addData("3 correction", correction);
             telemetry.update();
             
             driveStraight(0.3);
             
             if(gamepad1.a){
              rotate(90,0.3);
             }
            /**
            double r = Math.hypot(0, 0);
            double robotAngle = Math.atan2(0,0) - Math.PI / 4;
            double rightX = angles.firstAngle / 180;
            

            
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            front_left.setPower(0.0 - v1);
            front_right.setPower(v2);
            back_left.setPower(0.0 - v3);
            back_right.setPower(v4);
            */
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

private void driveStraight(double power)
{
// Use gyro to drive in a straight line.
 correction = checkDirection();
 /*
 double r = power;
 double robotAngle = degrees * (Math.PI / 180) - Math.PI / 4;
 double rightX = correction;
 
 final double v1 = r * Math.cos(robotAngle) - rightX;
 final double v2 = r * Math.sin(robotAngle) + rightX;
 final double v3 = r * Math.sin(robotAngle) - rightX;
 final double v4 = r * Math.cos(robotAngle) + rightX;
 */
 front_left.setPower(power + correction);
 front_right.setPower(power);
 back_left.setPower(power + correction);
 back_right.setPower(power);
 
}
 /**
 * Resets the cumulative angle tracking to zero.
 */
 private void resetAngle(double degrees)
 {
 lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
 wanted_direction += degrees;
 globalAngle = 0;
 }
 /**
 * Get current cumulative angle rotation from last reset.
 * @return Angle in degrees. + = left, - = right.
 */
 private double getAngle()
 {
 // We experimentally determined the Z axis is the axis we want to use for heading angle.
 // We have to process the angle because the imu works in euler angles so the Z axis is
 // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
 // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
 Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
AngleUnit.DEGREES);
 double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
 if (deltaAngle < -180){
 deltaAngle += 360;
 wanted_direction += 360;
 }
 else if (deltaAngle > 180){
 deltaAngle -= 360;
 wanted_direction -= 360;
 }
 globalAngle += deltaAngle;
 lastAngles = angles;
 return globalAngle;
 }
 /**
 * See if we are moving in a straight line and if not return a power correction value.
 * @return Power adjustment, + is adjust left - is adjust right.
 */
 private double checkDirection()
 {
 // The gain value determines how sensitive the correction is to direction changes.
 // You will have to experiment with your robot to get small smooth direction changes
 // to stay on a straight line.
 double correction, angle, gain = .10;
 angle = getAngle();
 if (angle == 0)
 correction = 0; // no adjustment.
 else
 correction = -angle; // reverse sign of angle for correction.
 correction = correction * gain;
 return correction;
 }
 /**
 * Rotate left or right the number of degrees. Does not support turning more than 180
degrees.
 * @param degrees Degrees to turn, + is left - is right
 */
 private void rotate(int degrees, double power)
 {
 double leftPower, rightPower;
 // restart imu movement tracking.
 //resetAngle();
 // getAngle() returns + when rotating counter clockwise (left) and - when rotating
 // clockwise (right).
 if (degrees < 0)
 { // turn right.
 leftPower = -power;
 rightPower = power;
 }
 else if (degrees > 0)
 { // turn left.
 leftPower = power;
 rightPower = -power;
 }
 else return;
 // set power to rotate.
 front_left.setPower(leftPower);
 back_left.setPower(leftPower);
 front_right.setPower(rightPower);
 back_right.setPower(rightPower);
 
 // rotate until turn is completed.
 if (degrees < 0)
 {
 // On right turn we have to get off zero first.
 while (opModeIsActive() && getAngle() == 0) {}
 while (opModeIsActive() && getAngle() > degrees) {}
 }
 else // left turn.
 while (opModeIsActive() && getAngle() < degrees) {}
 // turn the motors off.
 front_left.setPower(0);
 back_left.setPower(0);
 front_right.setPower(0);
 back_right.setPower(0);
 // wait for rotation to stop.
 sleep(1000);
 // reset angle tracking on new heading.
 resetAngle(degrees);
 }
    
}
