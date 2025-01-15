package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Mecanum_TeleOp_2024 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /*
         Declare our motors
         Make sure your ID's match your configuration
        */

        DcMotor lhook= hardwareMap.dcMotor.get("lhook");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor lift = hardwareMap.dcMotor.get("lift");

        //DcMotor rightLift = hardwareMap.dcMotor.get("rightLift");
        Servo drone = hardwareMap.servo.get("drone");
        Servo hook = hardwareMap.servo.get("hook");
        Servo claw = hardwareMap.servo.get("claw");
        Servo graber = hardwareMap.servo.get("graber");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; //* 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double driveSpeed = .6;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower*driveSpeed);
            motorBackLeft.setPower(backLeftPower*driveSpeed);
            motorFrontRight.setPower(frontRightPower*driveSpeed);
            motorBackRight.setPower(backRightPower*driveSpeed);
            if (gamepad1.y){
                lift.setPower(-1);
            }
            if (gamepad1.a){
                lift.setPower(1);
            }
            else{
                lift.setPower(0);
            }
            if (gamepad1.left_bumper){
                drone.setPosition(-10);
                Thread.sleep(1000);
                drone.setPosition(3);
                }
            if (gamepad1.left_bumper){
                claw.setPosition(-10);
                claw.setPosition(10);
                }
            if (gamepad1.right_bumper){
                //hook.setPosition(3);
                //Thread.sleep(1000);
                // hook.setPosition(-1);
                lhook.setPower(.5);
                }
            if (gamepad1.b){
                graber.setPosition(1);
                }
            if (gamepad1.x){
                graber.setPosition(-.5);
                }
            if (gamepad1.left_trigger>0){
                lhook.setPower(10);
                }
            if (gamepad1.right_trigger>0){
                lhook.setPower(-.5);
                }
            else{
                lhook.setPower(0);


            }
        }

    }
}




















