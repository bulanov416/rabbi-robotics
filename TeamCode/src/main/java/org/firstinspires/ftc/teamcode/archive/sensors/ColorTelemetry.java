package org.firstinspires.ftc.teamcode.archive.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Matan on 10/23/2016.
 */
@TeleOp(name="ColorTelemetry")
public class ColorTelemetry extends LinearOpMode {
    ColorSensor colorSensor;
    int red;
    int blue;
    String colorValues;
    public void runOpMode() throws InterruptedException {
        // in Java, while (true) can only exit by throwing an exception. This is not OK.
        // Instead, we use while (opModeIsActive()), which exits when the OpMode is closed.
        while (opModeIsActive()) {
            colorSensor = hardwareMap.colorSensor.get("color");

            colorValues = Integer.toString(colorSensor.argb());
            if (colorValues != "") { // this occurs when the beaconColor changes too quickly
                red = Integer.valueOf(colorValues.substring(2, 4));
                blue = Integer.valueOf(colorValues.substring(6, 8));
            } else {
                red = 0;
                blue = 0;
            }
            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("dominant beaconColor", red > blue ? "red" : "blue");
            telemetry.addData("Full beaconColor", colorValues);

            telemetry.update();
        }
    }
}

