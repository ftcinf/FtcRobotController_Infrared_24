//public void arm_mover (
//        double liftInches,
//        double volts,
//        double timeoutS
//        //double volts
//        ) {
//// the number 537.7 means,
//
//final double     COUNTS_PER_INCH         = (537.7 * 1.0) /
//        (4 * 3.1415);
//
//        telemetry.addData( "counts per inch ",COUNTS_PER_INCH);
//
//        // Ensure that the OpMode is still active
//        if (opModeIsActive()) {
//
//        // Determine new target position, and pass to motor controller
//        int currentarmposition = lift.getCurrentPosition() + (int) (liftInches * COUNTS_PER_INCH);
//        //double potentiometer_position = potentiometer.getVoltage();
//        // lift.setTargetPosition(currentarmposition);
//        lift.setPower(.8);
//
//        lift.setTargetPosition(currentarmposition);
//
//        telemetry.addData("the intialized position ", lift.getCurrentPosition());
//        telemetry.addData("the current arm position = ", currentarmposition);
//        telemetry.addData("desierd arm posiotion is,", liftInches);
//
//        // Turn On RUN_TO_POSITION
//
//        telemetry.addData("the current arm position = ", currentarmposition);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        runtime.reset();
//
//
//        // telemetry.addData("Potentiometer voltage",potentiometer.getVoltage());
//        while (opModeIsActive() &&
//        (runtime.seconds() < timeoutS) &&
//        (lift.isBusy())) {
//
//        telemetry.update();
//        }
//
//        lift.setPower(0);
//
//        // reset the timeout time and start motion.
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//        // Stop all motion;
//        int counter = 0;
//        // Double valueOf(String s);
//
//        double currentVoltage = potentiometer.getVoltage();
//// there was a 'public static' behind the 'final' but getting rid of them solved a error
//final DecimalFormat df = new DecimalFormat("0.00");
//
//        while (currentVoltage != volts && touchSensor.getState() == true && volts != 10) {
//
//        telemetry.addData("touch sensor state", touchSensor.getState());
//        telemetry.addData("loop count ", counter);
//        counter += 1;
//        //pot_difference allows us to scale how fast the arm
//        //moves into position based on how far away the currentvoltage is
//        //from the desired voltage
//        double pot_difference = (volts - currentVoltage)/volts;
//        //We are multiply by 100 because it will give us
//        //reasonable amounts of ticks to move the arm
//        //AND it is easy to do the math in our heads.
//        int ticks_to_move_arm = (int) (200 * pot_difference);
//
//        if (currentVoltage < volts) {
//        //up
//        telemetry.addData("up", "");
//        currentarmposition = currentarmposition +ticks_to_move_arm;
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(.8);
//        lift.setTargetPosition(currentarmposition);
//        }
//        else if (currentVoltage > volts) {
//        //down
//        telemetry.addData("down", "");
//        currentarmposition = currentarmposition + ticks_to_move_arm;
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(.8);
//        lift.setTargetPosition(currentarmposition);
//        }
//        else {
//        telemetry.addData("right", "position");
//        lift.setPower(0);
//        }
//
//        //Check if lift has finished moving
//        //if lift has not finished moving, the potentiometer reading
//        //will be out of sync with currentarmposition
//        //This will the currentarmposition to be set to nonsensical levels
//        while (lift.isBusy()){
//        // telemetry.addData("Lift is still moving... ", lift.isBusy());
//        // telemetry.update();
//        }
//        //s stores our rounded decimal but as a string
//        String s = df.format(potentiometer.getVoltage());
//        //Convert s string into dObj Double Value
//        Double dObj = Double.valueOf(s);
//        currentVoltage = dObj.doubleValue();
//        telemetry.addData("the current currentVoltage = ", currentVoltage);
//        telemetry.addData("the desired volts = ", volts);
//        telemetry.addData("the current arm position = ", currentarmposition);
//        telemetry.addData("ticks_to_move_arm = ", ticks_to_move_arm);
//        telemetry.update();
//        }
//
//        // Turn off RUN_TO_POSITION
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//        lift.setPower(0);
//        // optional pause after each move.
//        telemetry.addData("exited", "while loop");
//        telemetry.update();
//        }
//        }
