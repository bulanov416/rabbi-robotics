package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="Working Beacon Pusher")
public class WorkingBeaconPusher extends LinearOpMode {

    DcMotor l, r, lb, rb;
    Servo pusher;
    ColorSensor colorSensor;
    int red;
    int blue;
    String colorValues;
    int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;
    TouchSensor touchSensor;
    public WorkingBeaconPusher() {}

    public void runOpMode() throws InterruptedException {
        setupRobot(); // method for setting up our hardware

        waitForStart();

        // code that gets the robot to the beacon
        drive(1, 1);
        turnLeft(1, 1);
        drive(1, 0);
        waitForTouch();
        stopDriving();
        //pushButton();

    }

    public void waitForTouch() {
        while (!touchSensor.isPressed()) {

        }
    }

    public void drive(double power, double time) throws InterruptedException {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(power);
        lb.setPower(power);
        Thread.sleep((long) time * 100);
        stopDriving();
    }

    public void turnLeft(double power, double time) throws InterruptedException {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(-power);
        lb.setPower(-power);
        Thread.sleep((long) time * 100);
        stopDriving();
    }

    public void turnRight(double power, double time) throws InterruptedException {
        r.setPower(-power);
        rb.setPower(-power);
        l.setPower(power);
        lb.setPower(power);
        Thread.sleep((long) time * 100);
        stopDriving();
    }

    public void stopDriving() {
        r.setPower(0);
        rb.setPower(0);
        l.setPower(0);
        lb.setPower(0);
    }

    public void pushButton() throws InterruptedException {
        pusher.setPosition(rest_position);

        // returns string in this format: "aarrggb". for example "1924873409"
        colorValues = Integer.toString(colorSensor.argb());

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
            pusher.setPosition(beacon_left_button_pos);
        } else if (blue > red) {
            pusher.setPosition(beacon_right_button_pos);

        if (red > blue) {
            pusher.setPosition(160);
        } else if (blue > red) {
            pusher.setPosition(10);
        }
       }
    }

    public void setupRobot() {
        l  = hardwareMap.dcMotor.get("l");
        r  = hardwareMap.dcMotor.get("r");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        touchSensor = hardwareMap.touchSensor.get("touch");
        pusher = hardwareMap.servo.get("pusher");
        colorSensor = hardwareMap.colorSensor.get("color");
    }
}
