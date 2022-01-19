package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FirstJavaOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor rightDrive;
    private DcMotor leftDrive;
    private DcMotor sweep;
    private DcMotor lift;
    private Servo tilt; 

    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        lift = hardwareMap.get(DcMotor.class, "lift");
        sweep = hardwareMap.get(DcMotor.class, "sweep");
        tilt = hardwareMap.get(Servo.class, "tilt");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        double tiltPosition = 0.44;
        boolean sweepOn = false;
        
        telemetry.addData("status", "Initialized");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            tilt.setPosition(tiltPosition);
            
            rightDrive.setPower(gamepad1.right_stick_y);
            leftDrive.setPower(gamepad1.left_stick_y);
            
            lift.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            
            if (gamepad1.y) {
                tiltPosition = tiltPosition-0.0006;
                if (tiltPosition<0) {
                    tiltPosition = 0;
                }
            }
            if (gamepad1.x) {
                tiltPosition = tiltPosition+0.0006;
                if (tiltPosition>0.44) {
                    tiltPosition = 0.44;
                }
            }
            
            if (gamepad1.a) {
                if (sweepOn == false) {
                    sweep.setPower(1);
                    sweepOn = true;
                }
                else if (sweepOn == true) {
                    sweep.setPower(0);
                    sweepOn = false;
                }
            }
        }
    }
}
    