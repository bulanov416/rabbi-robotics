package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;

@Autonomous(name="Autonomous Button Pusher", group="AutoOps")
public class AutoButtonPusher extends LinearOpMode {

    DcMotor l, r, lb, rb;
    Servo pusher;
    ColorSensor colorSensor = hardwareMap.colorSensor.get("color");
    int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;
    int[] beacon_red = {256, 0, 0, 1};

    public AutoButtonPusher() {}

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
        int[] beacon_color = new int[4];
        boolean weAreRed = true;
        beacon_color[0] = colorSensor.red();
        beacon_color[1] = colorSensor.green();
        beacon_color[2] = colorSensor.blue();
        beacon_color[3] = colorSensor.alpha();

        /* This statement assumes that we are the RED team.
           This statement also assumes that the color sensor is looking at the left side of the beacon.
           TODO be like Leila Goodman and assume nothing. fix this if statement!
           You'll need to create a special blue class - we can't tell the OpMode which team we're on. */
        if (Arrays.equals(beacon_color, beacon_red)) {
            pusher.setPosition(beacon_left_button_pos);
            telemetry.addData("Pushed Button", "LEFT");
            telemetry.update();
        } else {
            pusher.setPosition(beacon_right_button_pos);
            telemetry.addData("Pushed button", "RIGHT");
            telemetry.update();
        }

        Thread.sleep(50);
        pusher.setPosition(Range.scale(rest_position, 0, 185, 0, 255));
        telemetry.addData("Button Push Successful. Release the triggers NOW.", "");
        telemetry.update();
        Thread.sleep(100);
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
