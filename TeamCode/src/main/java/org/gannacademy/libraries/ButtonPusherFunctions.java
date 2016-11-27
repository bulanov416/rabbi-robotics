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

    public Boolean robotIsOnWhite() {
        return robot.eods.getLightDetected() >= 0.38;
    }

    public void setPusherPosition() {
        robot.button_pusher.setPosition(rest_position);
        if (this.isSensorRed(0) && this.isSensorRed(0) != null) {
            robot.button_pusher.setPosition(HardwarePlatform.isRed ? beacon_left_button_pos : beacon_right_button_pos);
        }
        else if (!this.isSensorRed(0) && this.isSensorRed(0) != null) {
            robot.button_pusher.setPosition(HardwarePlatform.isRed ? beacon_right_button_pos : beacon_left_button_pos);
        }
    }

    public Boolean isSensorRed(int sensorNumber) {
        int red, blue;
        String colorValues = Integer.toString(sensorNumber == 0 ? robot.beaconColorA.argb() : robot.beaconColorB.argb()) ;
        if (colorValues == "") { // this occurs when the beaconColorA changes too quickly, returns null
            return null;
        } else { // extract the red and blue values from the string
            red = Integer.valueOf(colorValues.substring(2, 4));
            blue = Integer.valueOf(colorValues.substring(6, 8));
        }
        Boolean state = red > blue && blue != red ? true : false;
        return state;
    }

    public Boolean isBeaconPressed() {
        if (this.isSensorRed(0)!= null && this.isSensorRed(1) != null) {
            if (HardwarePlatform.isRed && this.isSensorRed(0) && this.isSensorRed(1) ) return true;
            else if ((!HardwarePlatform.isRed) && !(this.isSensorRed(0)) && !(this.isSensorRed(1))) return true;
            else return false;
        }
        else return null;
    }
}
