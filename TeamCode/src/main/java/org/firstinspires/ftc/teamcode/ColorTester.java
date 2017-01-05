package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by alexbulanov on 1/4/17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Color Tester")
public class ColorTester extends LinearOpMode {
    ColorSensor color_left;
    ColorSensor color_right;
    public void runOpMode() {
        String val_right;
        String val_left;
        while (true) {
            val_left = Integer.toString(color_left.argb());
            val_right = Integer.toString(color_right.argb());
            telemetry.addLine("Left: " + val_left);
            telemetry.addLine("Right: " + val_right);
            telemetry.update();
        }
    }
}
