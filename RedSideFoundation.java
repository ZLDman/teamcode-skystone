package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;
import java.util.List;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


@Autonomous

public class RedSideFoundation extends LinearOpMode{
  int COUNTS_PER_INCH = 22; //280 in a full turn
                            // 4 inch wheels
  DcMotor front_right;
  DcMotor front_left;
  DcMotor back_left;
  DcMotor back_right;
  
  ColorSensor dreaminColor;
  ModernRoboticsI2cRangeSensor homeOntheRange;
  
  Servo   foundation;
  Servo   foundation1;
  
    // todo: write your code here
    //@Override
    private ElapsedTime     runtime = new ElapsedTime();
    
        BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double wanted_direction = 0;
    
    
    public void runOpMode() {
        
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;
        
    front_left = hardwareMap.get(DcMotor.class,"front_left");
    front_right = hardwareMap.get(DcMotor.class,"front_right");
    back_left = hardwareMap.get(DcMotor.class,"back_left");
    back_right = hardwareMap.get(DcMotor.class,"back_right");
      
    foundation = hardwareMap.get(Servo.class, "foundation"); 
    foundation1 = hardwareMap.get(Servo.class, "foundation1");
     
    homeOntheRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "homeOntheRange");
    dreaminColor = hardwareMap.get(ColorSensor.class, "DreaminColor");
     
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
      
        waitForStart(); 
    
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    
   
      
//encoderDrive1(1,100,100,100,100,10);
//encoderDrive1(1,-100,100,-100,100,10);
//encoderDrive1(0.5,100,100,-100,-100,10);
//encoderDrive(1,-100,-100,10);
//encoderDrive(1,100,100,10);
//encoderDrive(1,-100,100,10);

//homeOntheRange.rawUltrasonic()
//Color.RGBToHSV((int) (dreaminColor.red() * SCALE_FACTOR),
//(int) (dreaminColor.green() * SCALE_FACTOR),
//(int) (dreaminColor.blue() * SCALE_FACTOR),
//hsvValues);




//open claws
foundation.setPosition(0);
foundation1.setPosition(0);


//drive forward until 85 cm from wall
front_left.setPower(0.5);
front_right.setPower(-0.5);
back_left.setPower(0.5);
back_right.setPower(-0.5);

sleep(2000);



//encoderDrive(0.5,-50,0,5);

stopMotors();

sleep(1000);

//clamp down on foundation
foundation.setPosition(0.5);
foundation1.setPosition(0.6);

sleep(1000);

front_left.setPower(-0.5);
front_right.setPower(0.5);
back_left.setPower(-0.5);
back_right.setPower(0.5);


while(homeOntheRange.rawUltrasonic() > 30 && opModeIsActive()){
    telemetry.addData("distance", homeOntheRange.rawUltrasonic());
    telemetry.update();
}

rotate(-90,0.5);

front_left.setPower(0.5);
front_right.setPower(-0.5);
back_left.setPower(0.5);
back_right.setPower(-0.5);

sleep(1000);

stopMotors();



foundation.setPosition(0);
foundation1.setPosition(0);

front_left.setPower(-0.5);
front_right.setPower(0.5);
back_left.setPower(-0.5);
back_right.setPower(0.5);

sleep(3000);

stopMotors();

front_left.setPower(0);
front_right.setPower(0);
back_left.setPower(0);
back_right.setPower(0);

telemetry.addData("done", " ");
telemetry.update();




//foundation.setPosition(0.75);

//encoderDrive(1,0,-210,10);

//foundation.setPosition(0);


      
}
    
public void stopMotors()
{
front_left.setPower(0);
front_right.setPower(0);
back_left.setPower(0);
back_right.setPower(0);
}

public void encoderDrive(double speed,double frontInches,double leftInches,double timeoutS) {
        
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double r = Math.hypot(0 - leftInches, frontInches);
        
        double robotAngle = Math.atan2(frontInches, 0 - leftInches) - Math.PI / 4;
        
        //double rightX = gamepad1.right_stick_x;
        
        final double v1 = r * (0 - Math.cos(robotAngle)); //-141
        final double v2 = r * Math.sin(robotAngle); //0
        final double v3 = r * (0 - Math.sin(robotAngle)); //0
        final double v4 = r * Math.cos(robotAngle); //141

        int newfront_leftTarget;
        int newfront_rightTarget;
        int newback_leftTarget;
        int newback_rightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfront_leftTarget = front_left.getCurrentPosition() + (int)(v1 * COUNTS_PER_INCH);
            newfront_rightTarget = front_right.getCurrentPosition() + (int)(v2 * COUNTS_PER_INCH);
            newback_leftTarget = back_left.getCurrentPosition() + (int)(v3 * COUNTS_PER_INCH);
            newback_rightTarget = back_right.getCurrentPosition() + (int)(v4 * COUNTS_PER_INCH);
            
            front_right.setTargetPosition(newfront_rightTarget);
            front_left.setTargetPosition(newfront_leftTarget);
            back_left.setTargetPosition(newback_leftTarget);
            back_right.setTargetPosition(newback_rightTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            back_left.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (front_right.isBusy() || front_left.isBusy() || back_left.isBusy() || back_right.isBusy())) {

                
            }

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
  
}
public void encoderDrive1(double speed,double frontleft, double frontright , double backleft, double backright ,double timeoutS) {
    front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    int newfrontrightTarget;
    int newfrontleftTarget;
    int newbackrightTarget;
    int newbackleftTarget;
    if (opModeIsActive()) {
        
        newfrontrightTarget = front_right.getCurrentPosition() + (int)(frontright * COUNTS_PER_INCH);
        newfrontleftTarget = front_left.getCurrentPosition() + (int)(frontleft * COUNTS_PER_INCH);
        newbackrightTarget = back_right.getCurrentPosition() + (int)(backright * COUNTS_PER_INCH);
        newbackleftTarget = back_left.getCurrentPosition() + (int)(backleft * COUNTS_PER_INCH);

        front_right.setTargetPosition(newfrontrightTarget);
        front_left.setTargetPosition(newfrontleftTarget);
        back_right.setTargetPosition(newbackrightTarget);
        back_left.setTargetPosition(newbackleftTarget);
        
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        runtime.reset();
        front_right.setPower(Math.abs(speed));
        front_left.setPower(Math.abs(speed));
        back_left.setPower(Math.abs(speed));
        back_right.setPower(Math.abs(speed));
        
        while (opModeIsActive() &&
        (runtime.seconds() < timeoutS) &&
        (front_right.isBusy() || front_left.isBusy() || back_left.isBusy() || back_right.isBusy())) {

                
        }
        
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }
}
private void driveStraight(double power)
{
    
    front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
 front_right.setPower(power - correction);
 back_left.setPower(power + correction);
 back_right.setPower(power - correction);
 
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
 double correction, angle, gain = .30;
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
     
front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

 double leftPower, rightPower;
 // restart imu movement tracking.
 //resetAngle();
 // getAngle() returns + when rotating counter clockwise (left) and - when rotating
 // clockwise (right).
 if (degrees < 0)
 { // turn right.
 leftPower = -power;
 rightPower = -power;
 }
 else if (degrees > 0)
 { // turn left.
 leftPower = power;
 rightPower = power;
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
