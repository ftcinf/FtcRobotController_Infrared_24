/*
        Copyright (c) 2024 Alan Smith
        All rights reserved.
        Redistribution and use in source and binary forms, with or without modification,
        are permitted (subject to the limitations in the disclaimer below) provided that
        the following conditions are met:
        Redistributions of source code must retain the above copyright notice, this list
        of conditions and the following disclaimer.
        Redistributions in binary form must reproduce the above copyright notice, this
        list of conditions and the following disclaimer in the documentation and/or
        other materials provided with the distribution.
        Neither the name of Alan Smith nor the names of its contributors may be used to
        endorse or promote products derived from this software without specific prior
        written permission.
        NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
        LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
        THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
        ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
        FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
        DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
        SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
        TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
        THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

/*
 * This OpMode illustrates how to use the REV Digital Indicator
 *
 * This is a simple way to add the REV Digital Indicator which has a red and green LED.
 * (and as you may remember, red + green = yellow so when they are both on you can get yellow)
 *
 * Why?
 * You can use this to show information to the driver.   For example, green might be that you can
 * pick up more game elements and red would be that you already have the possession limit.
 *
 * This OpMode assumes that the REV Digital Indicator is setup as 2 Digital Channels named
 * front_led_green and front_led_red. (the green should be the lower of the 2 channels it is plugged
 * into and the red should be the higher)
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * You can buy this product here:  https://www.revrobotics.com/rev-31-2010/
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "Concept: RevLED", group = "Concept")

public class Somthin_else extends OpMode {
    LED right_red;
    LED right_green;
    LED left_red;
    LED left_green;
    DistanceSensor right_sensor;
    DistanceSensor  left_sensor;
    static double right_sensor_reading;
    static double left_sensor_reading;
    @Override

    public void init() {
        right_green = hardwareMap.get(LED.class, "right_green");
        right_red = hardwareMap.get(LED.class, "right_red");
        left_green = hardwareMap.get(LED.class, "left_green");
        left_red = hardwareMap.get(LED.class, "left_red");
        right_sensor = hardwareMap.get(DistanceSensor.class, "right_sensor");
        left_sensor = hardwareMap.get(DistanceSensor.class, "left_sensor");
        right_sensor_reading = right_sensor.getDistance(DistanceUnit.INCH);
        left_sensor_reading = left_sensor.getDistance(DistanceUnit.INCH);
    }


    @Override
    public void loop() {
        if (right_sensor_reading< 4.1) {
            right_red.on();
            right_green.off();
        }else if (right_sensor_reading > 4){
            right_red.off();
            right_green.on();
        }
        if (left_sensor_reading< 4) {
            left_red.on();
            left_green.off();
        }else if(left_sensor_reading>4.1) {
            left_red.off();
            left_green.on();
        }
        right_sensor_reading = right_sensor.getDistance(DistanceUnit.INCH);
        left_sensor_reading = left_sensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("range", String.format("%.01f in", right_sensor.getDistance(DistanceUnit.INCH)));

    }
}
