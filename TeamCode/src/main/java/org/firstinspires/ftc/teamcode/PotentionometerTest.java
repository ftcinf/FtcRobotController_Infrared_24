package TelOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.lang.Double;


@TeleOp(name="THIS_ONE_test", group="Robot")
public class TestTelop extends LinearOpMode {
    private   DcMotor   lift=null;
    private AnalogInput  potentiometer;
    private DigitalChannel touchSensor;
    private ElapsedTime     runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration

        lift = hardwareMap.dcMotor.get("lift");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // graber.setZeroPowerBehavior(Servo.ZeroPowerBehavior.BRAKE);
        // graber2.setZeroPowerBehavior(Servo.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            if (gamepad1.left_bumper ){
                arm_mover(-50,.530, 60);
            }
            if (gamepad1.right_bumper ){
                arm_mover(-60,.970, 60);
            }



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

            sleep(3000);
            // lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
            telemetry.addData("exited","while loop");
            telemetry.update();

        }
    }

}




















