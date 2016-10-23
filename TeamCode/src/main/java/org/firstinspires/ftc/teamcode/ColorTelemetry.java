package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Matan on 10/23/2016.
 */
@TeleOp(name="ColorTelemetry")
public class ColorTelemetry extends LinearOpMode {
    ColorSensor colorSensor;
    public void runOpMode() {
        while (true) {
            colorSensor = hardwareMap.colorSensor.get("color");
            telemetry.addData("output", colorSensor);
            telemetry.update();
        }
    }
}
