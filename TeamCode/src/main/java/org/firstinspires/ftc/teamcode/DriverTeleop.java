package TelOp;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="2_driver_telop", group="Robot")
public class Extra_teleop extends LinearOpMode {
    private   DcMotor   lift=null;
    private AnalogInput  potentiometer;
    @Override
    public void runOpMode() throws InterruptedException {
//   double right_sensor_reading;
//   double left_sensor_reading;

        // Declare our motors
        // Make sure your ID's match your configuration

        //DcMotor lhook= hardwareMap.dcMotor.get("lhook");

        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor submersible_motor = hardwareMap.dcMotor.get("submersible_motor");


        lift = hardwareMap.dcMotor.get("lift");

        Servo graber = hardwareMap.servo.get("graber");
        Servo graber2 = hardwareMap.servo.get("graber2");
        DigitalChannel touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        // LED right_green = hardwareMap.get(LED.class, "right_green");
        // LED right_red = hardwareMap.get(LED.class, "right_red");
        // LED left_green = hardwareMap.get(LED.class, "left_green");
        // LED left_red = hardwareMap.get(LED.class, "left_red");
        // DistanceSensor right_sensor = hardwareMap.get(DistanceSensor.class, "right_sensor");
        // DistanceSensor left_sensor = hardwareMap.get(DistanceSensor.class, "left_sensor");



        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        //  right_sensor_reading = right_sensor.getDistance(DistanceUnit.INCH);
        // left_sensor_reading = left_sensor.getDistance(DistanceUnit.INCH);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // graber.setZeroPowerBehavior(Servo.ZeroPowerBehavior.BRAKE);
        // graber2.setZeroPowerBehavior(Servo.ZeroPowerBehavior.BRAKE);




        //Reverse the right side motors
        //Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; //* 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            double driveSpeed = -.6;

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
            if (gamepad2.y){
                lift.setPower(1);

            }
            else if (gamepad2.a){
                lift.setPower(-1);

            }
            else{
                lift.setPower(0);

            }
            //if (gamepad2.left_bumper){
            // submersible_motor.setPower(.8);

            // }
            // else if (gamepad2.right_bumper){
            // submersible_motor.setPower(-.8);

            // }
            // else{
            //  submersible_motor.setPower(0);

            // }


            if (gamepad2.b){
                graber.setPosition(1);
                graber2.setPosition(0);

            }
            if (gamepad2.x){
                graber.setPosition(0);
                graber2.setPosition(1);

            }
            if (touchSensor.getState() == false ){
                lift.setPower(.1);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                sleep(2000);

            }

            // if (gamepad1.right_bumper ){
            // arm_mover(-60,.97, 60);
            // }



//       if (right_sensor_reading< 4.1) {
//             right_red.on();
//              right_green.off();
//         }else if (right_sensor_reading > 9){
//              right_red.off();
//             right_green.on();
//         }
//         if (left_sensor_reading< 9) {
//             left_red.on();
//              left_green.off();
//         }else if(left_sensor_reading>4.1) {
//              left_red.off();
//             left_green.on();
//         }

            telemetry.addData("potentiometer voltage",potentiometer.getVoltage());

            telemetry.update();
            //potentiometer.getVoltage();




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




            // telemetry.addData("Potentiometer voltage",potentiometer.getVoltage());
            while (opModeIsActive() &&
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

            if (potentiometer.getVoltage() <volts) {
                //down
                lift.setPower(.1);
            }
            else if(potentiometer.getVoltage() >volts){
                lift.setPower(-.1);
            }
            else{
                lift.setPower(0);
            }



            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.


        }

    }
}


















