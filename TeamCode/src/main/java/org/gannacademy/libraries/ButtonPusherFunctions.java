package org.gannacademy.libraries;

import android.widget.Button;

/**
 * Created by Nathan on 11/21/2016.
 */
public class ButtonPusherFunctions {

    HardwarePlatform robot;

    private final int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;

    public ButtonPusherFunctions(HardwarePlatform hwPlatform) {
        robot = hwPlatform;
        robot.telemetry.addLine("Using ButtonPusherFunctions v1.0.0-161126");
        robot.telemetry.update();
    }

    public boolean robotIsOnWhite() {
        return robot.eods.getLightDetected() >= 0.38;
    }

    public void setPusherPosition() throws InterruptedException {
        int red, blue;
        robot.button_pusher.setPosition(rest_position);
        // returns string in this format: "aarrggb". for example "1924873409"
        String colorValues = Integer.toString(robot.beaconColor.argb());
        if (colorValues == "") { // this occurs when the beaconColor changes too quickly
            red = 0;
            blue = 0;
        } else {
            // extract the red and blue values from the string
            red = Integer.valueOf(colorValues.substring(2, 4));
            blue = Integer.valueOf(colorValues.substring(6, 8));
        }

        // this statement assumes that we are on the red team, and the sensor is on the left
        if (red > blue) {
            robot.button_pusher.setPosition(beacon_left_button_pos);
        } else if (blue > red) {
            robot.button_pusher.setPosition(beacon_right_button_pos);

            if (red > blue) {
                robot.button_pusher.setPosition(160);
            } else if (blue > red) {
                robot.button_pusher.setPosition(10);
            }
        }
    }
}
