package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 11/16/2016.
 */
@Autonomous(name="Turn from Line PoC", group="PoC")
// @Disabled
public class TurnFromLinePoC extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();
    double leftButtonPos = 45;
    double restPosition = 90;
    double rightButtonPos = 135;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Sensor Value", robot.eods.getRawLightDetected());
            telemetry.addData("Robot Drive", "Queued");
            robot.drive(-0.6);
            telemetry.addData("Robot Drive", "Started");
            while (robot.eods.getRawLightDetected() < 0.04) {
                telemetry.addData("Loop", "Running");
                Thread.sleep(60);
            }
            robot.stopDriving();
            robot.driveCentimeters(19.5, 0.6);
            robot.stopDriving();
            // turn until the line
            robot.setLeftPower(0.6);
            while (robot.eods.getRawLightDetected() < 0.04) {Thread.sleep(50);}
            robot.stopDriving();
            // drive up to the beacon
            robot.driveCentimeters(20, 0.6);
            // detect color and set servo
            /*pushButton();
            // drive up to the beacon and push it
            robot.driveCentimeters(5, 0.6);
            robot.stopDriving();
            Thread.sleep(1000)
            // drive away from the beacon
            robot.driveCentimeters(-12, 0.6);
            robot.turnRight(0.6, 0.5);
            // drive to the next beacon
            robot.drive(0.6);
            while (robot.eods.getLightDetected() < 0.37);
            // do some more stuff*/
            break;
        }
        stop();
    }

    public void pushButton() throws InterruptedException {

        int red, blue;
        robot.button_pusher.setPosition(restPosition);
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
            robot.button_pusher.setPosition(leftButtonPos);
        } else if (blue > red) {
            robot.button_pusher.setPosition(rightButtonPos);
        }
    }
}
