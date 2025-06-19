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


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double speed_y = -gamepad1.right_stick_y / 2.0;
            double speed_x = -gamepad1.right_stick_x / 2.0;

            if (gamepad1.right_stick_y != 0.0 ) {
                RightMotor.setPower(speed_y);
                LeftMotor.setPower(speed_y);
            }
            if (gamepad1.right_stick_x != 0.0){
                RightMotor.setPower(speed_x);
                LeftMotor.setPower(-speed_x);
            }

        }

    }
}




















