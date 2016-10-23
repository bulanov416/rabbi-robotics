package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.internal.testcode.TestColorSensors;

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
        while (opModeIsActive()) {
            colorSensor = hardwareMap.colorSensor.get("color");

            colorValues = Integer.toString(colorSensor.argb());
            red = Integer.valueOf(colorValues.substring(2, 3));
            blue = Integer.valueOf(colorValues.substring(6, 7));

            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("dominant color", red > blue ? "red" : "blue");
            telemetry.addData("all together now", colorSensor.argb());


            telemetry.update();
        }
    }
}

