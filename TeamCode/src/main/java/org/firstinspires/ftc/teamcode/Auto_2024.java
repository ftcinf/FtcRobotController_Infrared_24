

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.text.DecimalFormat;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_2024", group="Robot")
public class Auto_2024 extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotorEx         fleftDrive   = null;
    private DcMotorEx         frightDrive  = null;
    private DcMotorEx          bleftDrive   = null;
    private DcMotorEx       brightDrive  = null;
    private DcMotorEx       lift = null;
    private Servo               graber = null;
    private Servo               graber2 = null;
    private SparkFunOTOS myOtos;
    private ElapsedTime     runtime = new ElapsedTime();
    private DigitalChannel touchSensor;
    private AnalogInput  potentiometer;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //https://www.revrobotics.com/rev-41-1600/
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1000;
    static final double     TURN_SPEED              = 0.4;

    static final double     WHEEL_CIRCUMFERENCE_MM  = 104 * 3.14;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
    //static double currentheading;
    static SparkFunOTOS.Pose2D pos;


    @Override
    public void runOpMode() {



        // Initialize the drive system variables.
        fleftDrive  = hardwareMap.get(DcMotorEx .class, "motorFrontLeft");
        bleftDrive  = hardwareMap.get(DcMotorEx .class, "motorBackLeft");
        frightDrive = hardwareMap.get(DcMotorEx .class, "motorFrontRight");
        brightDrive = hardwareMap.get(DcMotorEx .class, "motorBackRight");
        lift = hardwareMap.get(DcMotorEx .class,"lift");
        graber = hardwareMap.get(Servo .class,"graber");
        graber2 = hardwareMap.get(Servo .class,"graber2");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        configureOtos();

        //currentheading = pos.h;







        // fDistance = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");

        // rDistance = hardwareMap.get(DistanceSensor.class, "right_distance_sensor");
        // lDistance = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        //lift = hardwareMap.dcMotor.get("lift");
        // graber = hardwareMap.servo.get("graber");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        fleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frightDrive.setDirection(DcMotor.Direction.FORWARD);
        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
        brightDrive.setDirection(DcMotor.Direction.FORWARD);



        fleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                fleftDrive.getCurrentPosition(),
                frightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //encoderDrive(speed, leftInches, rightInches, timeoutS)
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  22.5,  -22.5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, 12, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

        arm_mover(DRIVE_SPEED, 5, 5.0,4);
        graber.setPosition(1);
        graber2.setPosition(0);

        arm_mover(DRIVE_SPEED, -20, 5.0,4);

        encoderDrive(DRIVE_SPEED, -10, 10, 5.0);
        //from here down is in loop-ish

        arm_mover(DRIVE_SPEED, -43, 5.0,4);
        sleep(1000);
        encoderDrive(DRIVE_SPEED, -15, 15, 5.0);
        sleep(1000);
        graber.setPosition(0);
        graber2.setPosition(1);
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 25, -25, 5.0);
        //NEEDS TO BE FINALIZED
        turning(-40);
        arm_mover(DRIVE_SPEED, 63, 5.0,4);
        encoderDrive(DRIVE_SPEED, -45, 45, 5.0);
        turning(-175);
        encoderDrive(DRIVE_SPEED, -55, 55, 5.0);
        encoderDrive(DRIVE_SPEED, 10, -10, 5.0);

         arm_mover(DRIVE_SPEED,-20,5.0);
         encoderStrafe(DRIVE_SPEED, -10, 10, 5.0);
          graber.setPosition(1);
         graber2.setPosition(0);
         arm_mover(DRIVE_SPEED,-20,5.0);
         encoderDrive(DRIVE_SPEED, 15, -15, 5.0);
         turning(90);
         encoderDrive(DRIVE_SPEED, -41, 41, 5.0);
         turning(0);





// //to turn left both need to be negitive
// //to turn right both need to be positive





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    /**
     * @param speed
     * @param leftInches
     * @param rightInches
     * @param timeoutS
     */
    public void encoderDrive(double speed,
                             double leftInches,
                             double rightInches,
                             double timeoutS) {
        int newLeftTarget = (int)(610 * COUNTS_PER_MM);
        int newRightTarget = (int)(610 * COUNTS_PER_MM);
        double TPS = (537.7/ 60) * COUNTS_PER_WHEEL_REV;
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = fleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = bleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = brightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fleftDrive.setTargetPosition(newLeftTarget);
            frightDrive.setTargetPosition(newRightTarget);
            bleftDrive.setTargetPosition(newLeftTarget);
            brightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("speed",speed);
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            fleftDrive.setVelocity(Math.abs(speed));
            frightDrive.setVelocity(Math.abs(speed));
            bleftDrive.setVelocity(Math.abs(speed));
            brightDrive.setVelocity(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fleftDrive.isBusy() && frightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        fleftDrive.getCurrentPosition(), frightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fleftDrive.setVelocity(TPS);
            frightDrive.setVelocity(TPS);
            bleftDrive.setVelocity(TPS);
            brightDrive.setVelocity(TPS);

            // Turn off RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderStrafe(double speed,
                              double leftInches,
                              double rightInches,
                              double timeoutS) {
        int newbackLeftTarget = (int)(610 * COUNTS_PER_MM);
        int newfrontRightTarget = (int)(610 * COUNTS_PER_MM);
        int newbackRightTarget = (int)(610 * COUNTS_PER_MM);
        int newfrontLeftTarget = (int)(610 * COUNTS_PER_MM);
        double TPS = (537.7/ 60) * COUNTS_PER_WHEEL_REV;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = fleftDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbackLeftTarget = bleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackRightTarget = brightDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            fleftDrive.setTargetPosition(newfrontLeftTarget);
            frightDrive.setTargetPosition(newfrontRightTarget);
            bleftDrive.setTargetPosition(newbackLeftTarget);
            brightDrive.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("speed",speed);
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            fleftDrive.setVelocity(Math.abs(speed));
            frightDrive.setVelocity(Math.abs(speed));
            bleftDrive.setVelocity(Math.abs(speed));
            brightDrive.setVelocity(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fleftDrive.isBusy() && frightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newfrontLeftTarget,  newbackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        fleftDrive.getCurrentPosition(), frightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fleftDrive.setVelocity(TPS);
            frightDrive.setVelocity(TPS);
            bleftDrive.setVelocity(TPS);
            brightDrive.setVelocity(TPS);


            // Turn off RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.



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
            int currentarmposition = lift.getCurrentPosition() + (int) (liftInches * COUNTS_PER_INCH);
            //double potentiometer_position = potentiometer.getVoltage();
            // lift.setTargetPosition(currentarmposition);
            lift.setPower(.5);
            lift.setTargetPosition(currentarmposition);

            telemetry.addData("the intialized position ", lift.getCurrentPosition());
            telemetry.addData("the current arm position = ", currentarmposition);
            telemetry.addData("desierd arm posiotion is,", liftInches);

            // Turn On RUN_TO_POSITION

            telemetry.addData("the current arm position = ", currentarmposition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();


            // telemetry.addData("Potentiometer voltage",potentiometer.getVoltage());
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lift.isBusy())) {

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

            while (currentVoltage != volts && touchSensor.getState() == true) {

                telemetry.addData("touch sensor state", touchSensor.getState());
                telemetry.addData("loop count ", counter);
                counter += 1;
                if (currentVoltage < volts) {
                    //up
                    if currentarmposition

                    telemetry.addData("up", "");
                    lift.setPower(.1);
                    currentarmposition = currentarmposition - 1;
                    lift.setTargetPosition(currentarmposition);
                } else if (currentVoltage > volts) {
                    //down
                    telemetry.addData("down", "");
                    lift.setPower(-.1);
                    currentarmposition = currentarmposition + 1;
                    lift.setTargetPosition(currentarmposition);

                } else {
                    telemetry.addData("right", "position");
                    lift.setPower(0);
                }
                //s stores our rounded decimal but as a string
                while (lift.isBusy()){
                    telemetry.addData("Lift is still moving... ", lift.isBusy());
                }
                String s = df.format(potentiometer.getVoltage());
                //Convert s string into dObj Double Value
                Double dObj = Double.valueOf(s);
                currentVoltage = dObj.doubleValue();
                telemetry.addData("the current arm position = ", currentVoltage);
                telemetry.addData("the desiered arm position = ", volts);
                telemetry.addData("the current arm position = ", currentarmposition);
                telemetry.update();

            }

            // Turn off RUN_TO_POSITION


            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
            telemetry.addData("exited", "while loop");
            telemetry.update();
        }
        }
    public void turning(double heading){
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        // to ensure pos.h and heading can be equilvent, rounding pos.h and heading
        // to 1 decimal place


        while ((int)pos.h != (int)heading){

            if(pos.h > heading){
                fleftDrive.setVelocity(Math.abs(DRIVE_SPEED));
                frightDrive.setVelocity(Math.abs(DRIVE_SPEED));
                bleftDrive.setVelocity(Math.abs(DRIVE_SPEED));
                brightDrive.setVelocity(Math.abs(DRIVE_SPEED));
            }

            else if(pos.h < heading){
                fleftDrive.setVelocity(Math.abs(-DRIVE_SPEED));
                frightDrive.setVelocity(Math.abs(-DRIVE_SPEED));
                bleftDrive.setVelocity(Math.abs(-DRIVE_SPEED));
                brightDrive.setVelocity(Math.abs(-DRIVE_SPEED));
            }

            telemetry.addData("ext ext_pos_h:",  pos.h);

            pos = myOtos.getPosition();

            telemetry.addData("new ext_pos_h:",  pos.h);
            telemetry.update();

        }
        fleftDrive.setVelocity(0);
        frightDrive.setVelocity(0);
        bleftDrive.setVelocity(0);
        brightDrive.setVelocity(0);

    }

    private void configureOtos() {
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);


    }
}
