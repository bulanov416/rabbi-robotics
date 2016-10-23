package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="Working Beacon Pusher", group="AutoOps")
public class WorkingBeaconPusher extends LinearOpMode {

    DcMotor l, r, lb, rb;
    Servo pusher;
    ColorSensor colorSensor = hardwareMap.colorSensor.get("color");
    int red;
    int blue;
    String colorValues;
    int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;
    int[] beacon_red = {256, 0, 0, 1};

    public WorkingBeaconPusher() {}

    public void runOpMode() throws InterruptedException {
        setupRobot(); // method for setting up our hardware

        waitForStart();

        // code that runs after the start button is pressed

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
        colorValues = Integer.toString(colorSensor.argb());

        if (colorValues == "") {
            red = 0;
            blue = 0;
        } else {

            red = Integer.valueOf(colorValues.substring(2, 4));
            blue = Integer.valueOf(colorValues.substring(6, 8));
        }

            if (red > blue) {
                pusher.setPosition(/*fill*/);
            } else if (blue > red) {
                pusher.setPosition(/*fill*/);
            }
        }


    public void setupRobot() {
        l  = hardwareMap.dcMotor.get("l");
        r  = hardwareMap.dcMotor.get("r");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        pusher = hardwareMap.servo.get("pusher");
        colorSensor = hardwareMap.colorSensor.get("color");
        // set the beacon_red values here

    }
}
