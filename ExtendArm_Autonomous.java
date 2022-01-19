package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class ExtendArm_Autonomous extends LinearOpMode{
    
  private DcMotorSimple lift = null;
  private DcMotorSimple extend = null;    
    
  int COUNTS_PER_INCH = 22; //280 in a full turn
  int Skystone_placement = 0;                        // 4 inch wheels
  DcMotor front_right;
  DcMotor front_left;
  DcMotor back_left;
  DcMotor back_right;
    // todo: write your code here
    //@Override
    private ElapsedTime     runtime = new ElapsedTime();
    public void runOpMode() {
    front_left = hardwareMap.get(DcMotor.class,"front_left");
    front_right = hardwareMap.get(DcMotor.class,"front_right");
    back_left = hardwareMap.get(DcMotor.class,"back_left");
    back_right = hardwareMap.get(DcMotor.class,"back_right");
    
    extend  = hardwareMap.get(DcMotorSimple.class, "extend");
    lift = hardwareMap.get(DcMotorSimple.class, "lift");
      
      
      
    waitForStart();  
    
    //encoderDrive(1,0,200,10);
    
    //encoderDrive(1,100,0,10);
    
    sleep(25000);
    
    extend.setPower(-0.5);
    
    sleep(1000);
    
    extend.setPower(0);
    
    
    

    
    
    
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
}
