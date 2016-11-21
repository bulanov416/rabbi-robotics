package org.firstinspires.ftc.teamcode.archive.debug.poc;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Nathan.
 * Largely based on the SensorHTColor class in the examples folder.
 */
@Autonomous(name="Color Sensor Proof of Concept")
public class ColorSensorPoC extends LinearOpMode{

    ColorSensor colorSensor; // Variable for the HT Color Sensor

    public ColorSensorPoC() {}

    public void runOpMode() throws InterruptedException {
        // TODO make this into its own method
        // an array that will hold the hue, saturation, and value information
        float[] hsvValues = {0F, 0F, 0F};

        // for controlling the LED on the color sensor
        boolean bLedOn = true;

        // connect colorSensor to the HT Color Sensor. Make sure to check the name!
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        // so we know that the sensor is working
        colorSensor.enableLed(bLedOn);

        waitForStart(); // OpMode starts here

        while (opModeIsActive()) {
            // TODO write the code that does the things

            // convert color data to HSV and store it in hsvValues
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
            // create a string to format the RGB colors as (r, g, b, a)
            String formattedRgbaData = colorSensor.red() + " " + colorSensor.green() + " " +
                    colorSensor.blue() + " " + colorSensor.alpha();

            // Send the color data back to the DS
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("RGBA: ", formattedRgbaData);
            telemetry.addData("Hue", hsvValues[0]);
            // and update the data.
            telemetry.update();
        }
    }

}
