package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

@Autonomous(name = "Blue_Blocks_foundaton")

public class Blue_Blocks_Foundation_Combo extends LinearOpMode{
    private ColorSensor dreaminColor;
    private Blinker expansion_Hub_1;
    private ModernRoboticsI2cRangeSensor homeOntheRange;
    private ModernRoboticsI2cRangeSensor rightRange;
    private ModernRoboticsI2cRangeSensor leftRange;
    private DcMotor back_left;
    private DcMotor back_right;
    private Servo claw;
    private DcMotorSimple extend;
    private DcMotor front_left;
    private DcMotor front_right;
    private DigitalChannel high;
    private BNO055IMU imu;
    private DcMotorSimple lift;
    private DigitalChannel low;
    private Servo spin;
    private Servo capstone;
    private Servo   foundation;
    private Servo   foundation1;
    
    private double rightRangeValue;
    
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .50, correction;
    double wanted_direction = 0;
    
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    
    long travelDuration = 0;

    public void runOpMode() {

        // get a reference to our compass
        front_left = hardwareMap.get(DcMotor.class,"front_left");
        front_right = hardwareMap.get(DcMotor.class,"front_right");
        back_left = hardwareMap.get(DcMotor.class,"back_left");
        back_right = hardwareMap.get(DcMotor.class,"back_right");
        
        homeOntheRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "homeOntheRange");
        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");
        dreaminColor = hardwareMap.get(ColorSensor.class, "DreaminColor");
        
        extend = hardwareMap.get(DcMotorSimple.class, "extend");
        claw = hardwareMap.get(Servo.class, "claw");
        
        foundation = hardwareMap.get(Servo.class, "foundation"); 
        foundation1 = hardwareMap.get(Servo.class, "foundation1");
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
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
        
        //while(opModeIsActive()){
        //    telemetry.addData("Distance from target", rightRange.rawUltrasonic());
        //    telemetry.update();  
        //}
        //stopMotors();
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        while (homeOntheRange.rawUltrasonic() > 25  && opModeIsActive()){
            driveStraight(0.5);
            
            telemetry.addData("Distance from target", homeOntheRange.rawUltrasonic());
            telemetry.update();
        }
        stopMotors();
        
         Color.RGBToHSV((int) (dreaminColor.red() * SCALE_FACTOR),
            (int) (dreaminColor.green() * SCALE_FACTOR),
            (int) (dreaminColor.blue() * SCALE_FACTOR),
            hsvValues);
        
        while (hsvValues[0] < 90 && opModeIsActive()){
         Color.RGBToHSV((int) (dreaminColor.red() * SCALE_FACTOR),
            (int) (dreaminColor.green() * SCALE_FACTOR),
            (int) (dreaminColor.blue() * SCALE_FACTOR),
            hsvValues);
            
        telemetry.addData("Locking onto Target", hsvValues[0]);
        telemetry.update();    
        strafeRight(0.5);
        }
        stopMotors();
        
        rightRangeValue = rightRange.rawUltrasonic();
        sleep(10);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        //while (rightRange.rawUltrasonic() + 10 > rightRangeValue && opModeIsActive())
        //{
        //    telemetry.addData("Distance from Left Wall: ", rightRange.rawUltrasonic());
        //    telemetry.addData("Centering", "");
        //    telemetry.update();
        //    strafeRight(0.5);
        //}
        if (rightRangeValue > 66)
        {
            travelDuration = 1700;
            
            while (rightRange.rawUltrasonic() > 83 && opModeIsActive())
            strafeRight(0.5);
            stopMotors();
            
            while (rightRange.rawUltrasonic() < 81 && opModeIsActive())
            strafeLeft(0.5);
            stopMotors();
        }
        else if (rightRangeValue > 49)
        {
            travelDuration = 1800;
            
            while (rightRange.rawUltrasonic() > 64 && opModeIsActive())
            strafeRight(0.5);
            stopMotors();
            
            while (rightRange.rawUltrasonic() < 63 && opModeIsActive())
            strafeLeft(0.2);
            stopMotors();
            
            while (homeOntheRange.rawUltrasonic() < 30 && opModeIsActive())
            driveStraight(-0.3);
            stopMotors();   
        }
        else if (rightRangeValue > 31)
        {
            travelDuration = 2100;
            
            while (rightRange.rawUltrasonic() > 45 && opModeIsActive())
            strafeRight(0.2);
            stopMotors();
            
            while (rightRange.rawUltrasonic() < 43 && opModeIsActive())
            strafeLeft(0.2);
            stopMotors();
            
            while (homeOntheRange.rawUltrasonic() < 35 && opModeIsActive())
            driveStraight(-0.3);
            stopMotors();            
        }
        
        stopMotors();
        //sleep(2000);
        
        ////START AT ~1.5" LIFT
        
        extend.setPower(-0.5);
        sleep(1000);
        extend.setPower(0);
        claw.setPosition(1);
        
        
        while (homeOntheRange.rawUltrasonic() > 15 && opModeIsActive())
        {
            driveStraight(0.5);
            telemetry.addData("Approaching", "Prep");
            telemetry.update();
        }
        stopMotors();
        claw.setPosition(0);
        sleep(750);
        driveStraight(-0.2);
        sleep(1000);
        stopMotors();
        rotate(-80, 0.75);
        
        //while (rightRange.rawUltrasonic() < 80 && opModeIsActive())
        //{
            //strafeLeft(0.5);
            //telemetry.addData("Clearing a Path", "");
            //telemetry.update();
        //}
        stopMotors();
        driveStraight(1);
        sleep(travelDuration + 1000); //time for 4th position is 2000
        stopMotors();
        
        
        rotate(80, 0.75); // TBD
        
        
        
        extend.setPower(-0.5);
        sleep(1000);
        extend.setPower(0);
        
        

        
        
        
        claw.setPosition(1);
        
        
        
        
        
        
        
        extend.setPower(0.25);
        rotate(170,0.75); //TBD
        extend.setPower(0);
        
        
        
        foundation.setPosition(0);
        foundation1.setPosition(0);
        
        
        //drive forward until 85 cm from wall
        front_left.setPower(-0.5);
        front_right.setPower(-0.5);
        back_left.setPower(-0.5);
        back_right.setPower(-0.5);
        
        sleep(750);
        
        
        
        //encoderDrive(0.5,-50,0,5);
        
        stopMotors();
        
        sleep(500);
        
        //clamp down on foundation
        foundation.setPosition(0.5);
        foundation1.setPosition(0.6);
        
        sleep(1000);
        
        front_left.setPower(0.5);
        front_right.setPower(0.5);
        back_left.setPower(0.5);
        back_right.setPower(0.5);
        
        
        while(homeOntheRange.rawUltrasonic() > 30 && opModeIsActive()){
            telemetry.addData("distance", homeOntheRange.rawUltrasonic());
            telemetry.update();
        }
        
        rotate(-90,0.5);
        
        //front_left.setPower(-0.5);
        //front_right.setPower(-0.5);
        //back_left.setPower(-0.5);
        //back_right.setPower(-0.5);
        
        //sleep(1000);
        
        stopMotors();
        
        
        
        foundation.setPosition(0);
        foundation1.setPosition(0);
        
        driveStraight(-1);
        sleep(250);
        stopMotors();
        
        front_left.setPower(0.75);
        front_right.setPower(0.75);
        back_left.setPower(0.75);
        back_right.setPower(0.75);
        
        sleep(1500);
        
        stopMotors();
        
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

    }
    
    
    
    
    
    
    
    
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~Reset Angle~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
     private void resetAngle()
     {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            globalAngle = 0;
     }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~Drive Straight~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
      private void driveStraight(double power){
        // Use gyro to drive in a straight line.
        correction = checkDirection();
         /*double r = power;
         double robotAngle = degrees - Math.PI / 4;
         double rightX = correction;
     
         final double v1 = r * Math.cos(robotAngle) + rightX;
         final double v2 = r * Math.sin(robotAngle) - rightX;
         final double v3 = r * Math.sin(robotAngle) + rightX;
         final double v4 = r * Math.cos(robotAngle) - rightX;
      */     
         front_left.setPower(power - correction);
         front_right.setPower(power + correction);
         back_left.setPower(power - correction);
         back_right.setPower(power + correction);
     }
     /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~Check Direction~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
     private double checkDirection() {
         double correction, angle, gain = .10;
        angle = getAngle();
         if (angle == 0)
         correction = 0;
         else
         correction = -angle;
         correction = correction * gain;
         return correction;
    }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~Get Angle~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
    private double getAngle()
        {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
        deltaAngle += 360;
        else if (deltaAngle > 180)
        deltaAngle -= 360;
        
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~Rotate~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
    private void rotate(int degrees, double power)
        {
            degrees *= -1;
       double leftPower, rightPower;
            // restart imu movement tracking.
            resetAngle();
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).
            if (degrees < 0)
            { // turn right
            leftPower = power;
            rightPower = -power;
               }
       else if (degrees > 0)
           { // turn left
           leftPower = -power;
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
           while (opModeIsActive() && getAngle() > degrees) {telemetry.addData("Robot Angle: ", getAngle()); telemetry.update();}
           }
           else // left turn.
           while (opModeIsActive() && getAngle() < degrees) {telemetry.addData("Robot Angle: ", getAngle()); telemetry.update();}
           // turn the motors off.
           front_left.setPower(0);
           back_left.setPower(0);
           front_right.setPower(0);
           back_right.setPower(0);
           // wait for rotation to stop.
           sleep(250);
           // reset angle tracking on new heading.
           resetAngle();
           }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~Stop Motors~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
    private void stopMotors(){
        front_left.setPower(0);
         front_right.setPower(0);
         back_left.setPower(0);
         back_right.setPower(0);
    }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~Strafe Right~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
    private void strafeRight(double power){
        // Use gyro to strafe in a straight line.
        correction = checkDirection();
         /*double r = power;
         double robotAngle = degrees - Math.PI / 4;
         double rightX = correction;
     
         final double v1 = r * Math.cos(robotAngle) + rightX;
         final double v2 = r * Math.sin(robotAngle) - rightX;
         final double v3 = r * Math.sin(robotAngle) + rightX;
         final double v4 = r * Math.cos(robotAngle) - rightX;
      */     
         front_left.setPower(power - correction);
         front_right.setPower(-power - correction);
         back_left.setPower(-power + correction);
         back_right.setPower(power + correction);
    }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~Strafe Left~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
    private void strafeLeft(double power){
        // Use gyro to strafe in a straight line.
        correction = checkDirection();
         /*double r = power;
         double robotAngle = degrees - Math.PI / 4;
         double rightX = correction;
     
         final double v1 = r * Math.cos(robotAngle) + rightX;
         final double v2 = r * Math.sin(robotAngle) - rightX;
         final double v3 = r * Math.sin(robotAngle) + rightX;
         final double v4 = r * Math.cos(robotAngle) - rightX;
      */     
         front_left.setPower(-power - correction);
         front_right.setPower(power  + 0.5 + correction);
         back_left.setPower(power + 0.5 - correction);
         back_right.setPower(-power + correction);
    }
    /*
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~Obtaining~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
}
