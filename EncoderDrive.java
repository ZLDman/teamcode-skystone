package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class EncoderDrive extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor rightDrive;
    private DcMotor leftDrive;
    private DcMotor sweep;
    private DcMotor lift;
    private Servo tilt; 
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 288;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        sweep = hardwareMap.get(DcMotor.class, "sweep");
        lift = hardwareMap.get(DcMotor.class, "lift");
        tilt = hardwareMap.get(Servo.class, "tilt");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
    }
    
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;
        
        if (opModeIsActive()) {
            encoderDrive(1, 12, 12, 5);
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches*COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches*COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            
            while (opModeIsActive() && (runtime.seconds()<timeoutS) && (leftDrive.isBusy() && rightDrive.isBusy()))
            {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
      
    }
}
