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
    public void runOpMode() {
        while (true) {
            colorSensor = hardwareMap.colorSensor.get("color");
            telemetry.addData("red", colorSensor.red());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.addData("green", colorSensor.green());
            telemetry.addData("alpha", colorSensor.alpha());
            telemetry.addData("all together now", colorSensor.argb());
            telemetry.update();
        }
    }
}
