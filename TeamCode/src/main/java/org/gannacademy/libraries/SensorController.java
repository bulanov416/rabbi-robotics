package org.gannacademy.libraries;

/**
 * Created by Nathan on 11/21/2016.
 */
public class SensorController {

    HardwarePlatform robot;


    public SensorController(HardwarePlatform hardwarePlatform) {
        robot = hardwarePlatform;
        robot.telemetry.addLine("Using SensorController v2.0.0-161126");
        robot.telemetry.update();
    }

    public double getReflectedLight() {return robot.eods.getLightDetected();}
}
