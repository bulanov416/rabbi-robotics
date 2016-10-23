package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Working Beacon Pusher", group="AutoOps")
public class WorkingBeaconPusher extends LinearOpMode {

    DcMotor l, r, lb, rb;
    Servo pusher;
    ColorSensor colorSensor;
    int red;
    int blue;
    String colorValues;
    int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;

    public WorkingBeaconPusher() {}

    public void runOpMode() throws InterruptedException {
        setupRobot(); // method for setting up our hardware

        waitForStart();

        // code that gets the robot to the beacon

        pushButton();
    }

    public void drive(double power, double time) throws InterruptedException {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(power);
        lb.setPower(power);
        Thread.sleep((long) time * 100);
        stopDriving();
    }

    public void turn(double power, double time) throws InterruptedException {
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
        colorValues = Integer.toString(colorSensor.argb());

        if (colorValues == "") {
            red = 0;
            blue = 0;
        } else {

            red = Integer.valueOf(colorValues.substring(2, 4));
            blue = Integer.valueOf(colorValues.substring(6, 8));
        }
            // this statement assumes that we are on the red team, and the sensor is on the left
            if (red > blue) {
                pusher.setPosition(beacon_left_button_pos);
            } else if (blue > red) {
                pusher.setPosition(beacon_right_button_pos);
            }
        }


    public void setupRobot() {
        l  = hardwareMap.dcMotor.get("l");
        r  = hardwareMap.dcMotor.get("r");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        pusher = hardwareMap.servo.get("pusher");
        colorSensor = hardwareMap.colorSensor.get("color");
    }
}
