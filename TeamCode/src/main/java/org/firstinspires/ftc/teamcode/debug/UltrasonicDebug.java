package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Levi on 11/9/2016.
 */
@TeleOp(name="UltrasonicDebug")
public class UltrasonicDebug extends LinearOpMode {

    public UltrasonicDebug() {

    }

    @Override
    public void runOpMode() {
        UltrasonicSensor us = hardwareMap.ultrasonicSensor.get("us");
        waitForStart();
        while (opModeIsActive()) {
            if (System.currentTimeMillis() % 100 == 0) {
                telemetry.addLine("Current Level: " + us.getUltrasonicLevel());
                telemetry.update();
            }
        }
    }
}
