package org.gannacademy.libraries;

/**
 * Created by Nathan on 11/21/2016.
 */
public class ButtonPusherFunctions {

    HardwarePlatform robot;

    private final int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;

    public ButtonPusherFunctions(HardwarePlatform hwPlatform) {
        robot = hwPlatform;
        robot.telemetry.addLine("Using ButtonPusherFunctions v1.1.0-161126");
        robot.telemetry.update();
    }

    public boolean robotIsOnWhite() {
        return robot.eods.getLightDetected() >= 0.38;
    }

    public void setPusherPosition() {
        robot.button_pusher.setPosition(rest_position);
        if (this.isSensorRed(0)) {
            robot.button_pusher.setPosition(HardwarePlatform.isRed ? beacon_left_button_pos : beacon_right_button_pos);
        }
        else if (!this.isSensorRed(0)) {
            robot.button_pusher.setPosition(HardwarePlatform.isRed ? beacon_right_button_pos : beacon_left_button_pos);
        }
    }

    public boolean isSensorRed(int sensorNumber) {
        int red, blue;
        String colorValues = Integer.toString(sensorNumber == 0 ? robot.beaconColorA.argb() : robot.beaconColorB.argb()) ;
        if (colorValues == "") { // this occurs when the beaconColorA changes too quickly, returns null
            return new Boolean(null);
        } else { // extract the red and blue values from the string
            red = Integer.valueOf(colorValues.substring(2, 4));
            blue = Integer.valueOf(colorValues.substring(6, 8));
        }
        boolean state = red > blue && blue != red ? true : false;
        return state;
    }

    public boolean isBeaconPressed() {
        if (HardwarePlatform.isRed && this.isSensorRed(0) && this.isSensorRed(1) ) return true;
        else if ((!HardwarePlatform.isRed) && !(this.isSensorRed(0)) && !(this.isSensorRed(1))) return false;
        else return false;
    }
}
