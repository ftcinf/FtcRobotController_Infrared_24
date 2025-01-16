package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.text.DecimalFormat;

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
        public void arm_mover (
        double liftInches,
        double volts,
        double timeoutS
        //double volts
    ) {
            // the number 537.7 means,

            final double     COUNTS_PER_INCH         = (537.7 * 1.0) /
                    (4 * 3.1415);

            telemetry.addData( "counts per inch ",COUNTS_PER_INCH);

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                int currentarmposition = lift.getCurrentPosition() + (int)(liftInches * COUNTS_PER_INCH);
                //double potentiometer_position = potentiometer.getVoltage();
                // lift.setTargetPosition(currentarmposition);
                lift.setPower(.5);
                lift.setTargetPosition(currentarmposition);

                telemetry.addData("the intialized position ",lift.getCurrentPosition());
                telemetry.addData("the current arm position = ",currentarmposition);
                telemetry.addData("desierd arm posiotion is,",liftInches);

                // Turn On RUN_TO_POSITION

                telemetry.addData("the current arm position = ",currentarmposition);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();



                // telemetry.addData("Potentiometer voltage",potentiometer.getVoltage());
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (lift.isBusy() )) {

                    telemetry.update();
                }

                lift.setPower(0);

                // reset the timeout time and start motion.
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                // Stop all motion;
                int counter = 0;
                // Double valueOf(String s);

                double currentVoltage = potentiometer.getVoltage();
                // there was a 'public static' behind the 'final' but getting rid of them solved a error
                final DecimalFormat df = new DecimalFormat("0.00");

                while(currentVoltage != volts &&  touchSensor.getState()== true){

                    telemetry.addData("touch sensor state",touchSensor.getState());
                    telemetry.addData("loop count ",counter);
                    counter += 1;
                    if (currentVoltage < volts) {
                        //up
                        telemetry.addData("up","");
                        lift.setPower(.1);
                        currentarmposition = currentarmposition - 1;
                        lift.setTargetPosition(currentarmposition);
                    }
                    else if(currentVoltage > volts){
                        //down
                        telemetry.addData("down","");
                        lift.setPower(-.1);
                        currentarmposition = currentarmposition + 1;
                        lift.setTargetPosition(currentarmposition);

                    }
                    else{
                        telemetry.addData("right","position");
                        lift.setPower(0);
                    }
                    //s stores our rounded decimal but as a string
                    String s = df.format(potentiometer.getVoltage());
                    //Convert s string into dObj Double Value
                    Double dObj = Double.valueOf(s);
                    currentVoltage = dObj.doubleValue();
                    telemetry.addData("the current arm position = ",currentVoltage);
                    telemetry.addData("the desiered arm position = ",volts);
                    telemetry.addData("the current arm position = ",currentarmposition);
                    telemetry.update();

                }

                // Turn off RUN_TO_POSITION


                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                sleep(250);   // optional pause after each move.
                telemetry.addData("exited","while loop");
                telemetry.update();

            }
    }
}




















