package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OutreachRobotCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration



        DcMotor RightMotor = hardwareMap.dcMotor.get("RightMotor");
        DcMotor LeftMotor = hardwareMap.dcMotor.get("LeftMotor");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double speed = -gamepad1.right_stick_y / 2.0;

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            if (gamepad1.right_stick_y != 0.0 ) {
                RightMotor.setPower(speed);
                LeftMotor.setPower(speed);
            }
            if (gamepad1.right_stick_x != 0.0){
                RightMotor.setPower(speed);
                LeftMotor.setPower(-speed);
            }

        }

    }
}




















