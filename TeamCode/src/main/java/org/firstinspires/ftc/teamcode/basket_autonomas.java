

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;





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

@Autonomous(name="this one red multi", group="Robot")
public class This_one_red_multy extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotorEx         fleftDrive   = null;
    private DcMotorEx         frightDrive  = null;
    private DcMotorEx          bleftDrive   = null;
    private DcMotorEx          brightDrive  = null;
    private DistanceSensor rDistance;
    private DistanceSensor lDistance;
    private DistanceSensor fDistance;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //https://www.revrobotics.com/rev-41-1600/
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 500;
    static final double     TURN_SPEED              = 0.4;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fleftDrive  = hardwareMap.get(DcMotorEx .class, "motorFrontLeft");
        bleftDrive  = hardwareMap.get(DcMotorEx .class, "motorBackLeft");
        frightDrive = hardwareMap.get(DcMotorEx .class, "motorFrontRight");
        brightDrive = hardwareMap.get(DcMotorEx .class, "motorBackRight");
        // fDistance = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");
        // rDistance = hardwareMap.get(DistanceSensor.class, "right_distance_sensor");
        // lDistance = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        // DcMotor lift = hardwareMap.dcMotor.get("lift");
        //  Servo graber = hardwareMap.servo.get("graber");
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

        encoderDrive(DRIVE_SPEED,  -28,  28, 5.0);
        //lower arm
        //spin intake out
        //lift arm up
        encoderDrive(DRIVE_SPEED,  28,  -28, 5.0);


//to turn left both need to be negitive
//to turn right both need to be positive





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





}
