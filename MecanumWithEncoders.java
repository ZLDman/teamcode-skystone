package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp

public class Mecanum extends LinearOpMode{
  
  //spin servo
  
  int pushButton;
  
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotorSimple lift = null;
  private DcMotorSimple extend = null;
  
  static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
  static final int    CYCLE_MS    =   10;     // period of each cycle
  static final double MAX_POS     =  1;     // Maximum rotational position
  static final double MIN_POS     =  0.25;     // Minimum rotational position

  
  DcMotor front_right;
  DcMotor front_left;
  DcMotor back_left;
  DcMotor back_right;
  
  Servo   spin;
  Servo   claw;
  Servo   foundation;
  Servo   foundation1;
  Servo   capstone;
  
  DigitalChannel high;  // Hardware Device Object
  DigitalChannel low;
  
      
    double  position = 0.65;//(MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
  
    // todo: write your code here
    @Override
    public void runOpMode() {
          
    spin = hardwareMap.get(Servo.class, "spin");
    claw = hardwareMap.get(Servo.class, "claw"); 
    foundation = hardwareMap.get(Servo.class, "foundation"); 
    foundation1 = hardwareMap.get(Servo.class, "foundation1");
    capstone = hardwareMap.get(Servo.class,"capstone");
      
    front_left = hardwareMap.get(DcMotor.class,"front_left");
    front_right = hardwareMap.get(DcMotor.class,"front_right");
    back_left = hardwareMap.get(DcMotor.class,"back_left");
    back_right = hardwareMap.get(DcMotor.class,"back_right");
    
    extend  = hardwareMap.get(DcMotorSimple.class, "extend");
    lift = hardwareMap.get(DcMotorSimple.class, "lift");

    high = hardwareMap.get(DigitalChannel.class, "high");
    low = hardwareMap.get(DigitalChannel.class, "low");  
     
    high.setMode(DigitalChannel.Mode.INPUT);
    low.setMode(DigitalChannel.Mode.INPUT); 
     
    waitForStart();  

        
        
    while (opModeIsActive()) {
        
        if(gamepad1.a){
            capstone.setPosition(0.75);
        }
        if(gamepad1.b){
            capstone.setPosition(0);
        }
        
        if(gamepad2.right_trigger > 0){
            extend.setPower(-gamepad2.right_trigger / 2);
        }
        else{
            extend.setPower(gamepad2.left_trigger / 2);
        }
        
        
        //lift.setPower(-gamepad2.left_stick_y);
        
        /*
        if(high.getState() == false || low.getState() == false)
        {
            lift.setPower(0);
        }
        else
        {
            lift.setPower(-gamepad2.left_stick_y);
        }
        
        */
        
        if (high.getState() == false) {
            pushButton = -1;
        } 
        else if(low.getState() == false){
            pushButton = 1;
        }
        else{
            pushButton = 0;
        }

        
        
        if(gamepad2.left_stick_y < 0 && pushButton == -1){
           lift.setPower(0);
        }
        else if(gamepad2.left_stick_y > 0 && pushButton == 1){
           lift.setPower(0);
        }
        else{
            lift.setPower(-gamepad2.left_stick_y);
        }
        
        telemetry.addData("pushButton",pushButton);
            
        telemetry.update();
        
          
        if(gamepad1.left_bumper){
            foundation.setPosition(0.5);
            foundation1.setPosition(0.6);
        }  
        if(gamepad1.right_bumper){
            foundation.setPosition(0);
            foundation1.setPosition(0);
        }  
          
        
        position -= gamepad2.right_stick_x / 250;
                
        if(position > 1){
            position = 1;
        }
        else if(position < 0){
            position = 0;
        }
        
        // Set the servo to the new position and pause;
        spin.setPosition(position);
        if(gamepad2.left_bumper){
            claw.setPosition(0);
        }
        if(gamepad2.right_bumper){
            claw.setPosition(1);
        }
    
          
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        
        if(gamepad1.left_trigger > 0.5){
            r = r / 2;
            rightX = rightX / 2;
        }
        
        if(gamepad1.right_trigger > 0.5){
            r = r / 4;
            rightX = rightX / 4;
        }
        
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        
        
        front_left.setPower(0.0 - v1);
        front_right.setPower(v2);
        back_left.setPower(0.0 - v3);
        back_right.setPower(v4);
     
        }
    }
}
