package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.gannacademy.libraries.HardwareRabbi;

@Autonomous(name="Auto Beacon Pusher")
public class BeaconPusherAuto extends LinearOpMode {

    private HardwareRabbi robot = new HardwareRabbi();

    private final int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;

    public BeaconPusherAuto() {}

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        // code that gets the robot to the beacon
        robot.driveTicks(1, 1);
        robot.turnLeftTicks(1, 1);
        robot.driveTicks(1, 1);
        robot.turnLeftTicks(1, 1);
        // TODO we need a system to decide which side of the beacon to push
        // we need more detailed info on hardware for this
        robot.buttonPushServo.setPosition(rest_position); // TODO find correct values for each side
        waitForTouch();
        pushButton();
        telemetry.addData("Beacon Push", "SUCCESSFUL");

    }

    public void waitForTouch() {

        // TODO find a better way for this - empty while loops are terrible practice.
        while (!robot.buttonPusherTouch.isPressed()) {}

    }

    public void pushButton() throws InterruptedException {

            int red, blue;
            robot.buttonPushServo.setPosition(rest_position);
            // returns string in this format: "aarrggb". for example "1924873409"
            String colorValues = Integer.toString(robot.color.argb());
            if (colorValues == "") { // this occurs when the color changes too quickly
                red = 0;
                blue = 0;
            } else {
                // extract the red and blue values from the string
                red = Integer.valueOf(colorValues.substring(2, 4));
                blue = Integer.valueOf(colorValues.substring(6, 8));
            }

            // this statement assumes that we are on the red team, and the sensor is on the left
            if (red > blue) {
                robot.buttonPushServo.setPosition(beacon_left_button_pos);
            } else if (blue > red) {
                robot.buttonPushServo.setPosition(beacon_right_button_pos);

                if (red > blue) {
                    robot.buttonPushServo.setPosition(160);
                } else if (blue > red) {
                    robot.buttonPushServo.setPosition(10);
                }
            }
    }

    public void scaleServoValues() {
        Range.scale(beacon_left_button_pos, 0, 180, 0, 255);
        Range.scale(beacon_right_button_pos, 0, 180, 0, 255);
        Range.scale(rest_position, 0, 180, 0, 255);
        telemetry.addData("Beacon Values", "SET");
        telemetry.update();
    }
}
