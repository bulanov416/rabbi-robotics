package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Levi on 11/9/2016.
 */
@TeleOp(name="LightDebug")
public class LightDebug extends LinearOpMode {

    public LightDebug() {

    }

    @Override
    public void runOpMode() {
        LightSensor ls = hardwareMap.lightSensor.get("ls");
        ls.enableLed(true);
        telemetry.addLine("Max Raw Light: " + ls.getRawLightDetectedMax());
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (System.currentTimeMillis() % 100 == 0) {
                telemetry.addLine("Light Sensor Reading: " + ls.getLightDetected());
                telemetry.addLine("Raw Light Reading: " + ls.getRawLightDetected());
                telemetry.update();
            }
        }
    }
}
