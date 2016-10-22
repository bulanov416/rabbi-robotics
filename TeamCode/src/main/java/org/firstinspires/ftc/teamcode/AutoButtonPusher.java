package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

/**
 * Created by Nathan on 10/19/2016.
 */
@Autonomous(name="Autonomous Button Pusher", group="AutoOps")
public class AutoButtonPusher extends LinearOpMode {

    DcMotor l, r, lb, rb;
    Servo buttonpusher;
    ColorSensor colorSensor;
    int beacon_left_button_pos = 135, beacon_right_button_pos = 45, rest_position = 90;
    int[] beaconRed = new int[4];

    public AutoButtonPusher() {}

    public void runOpMode() {

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
        int[] beaconColor = new int[4];
        boolean weAreRed = true;
        beaconColor[0] = colorSensor.red();
        beaconColor[1] = colorSensor.green();
        beaconColor[2] = colorSensor.blue();
        beaconColor[3] = colorSensor.alpha();

        if (Arrays.equals(beaconColor, beaconRed) & weAreRed) {
            buttonpusher.setPosition(Range.scale(beacon_left_button_pos, 0, 185, 0, 255));
            telemetry.addData("Pushing left button", "");
            telemetry.update();
        } else if (Arrays.equals(beaconColor, beaconRed) & !weAreRed){
            buttonpusher.setPosition(Range.scale(beacon_right_button_pos, 0, 185, 0, 255));
            telemetry.addData("Pushing right button", "");
            telemetry.update();
        } else if (!Arrays.equals(beaconColor, beaconRed) & weAreRed) {
            buttonpusher.setPosition(Range.scale(beacon_right_button_pos, 0, 185, 0, 255));
            telemetry.addData("Pushing right button", "");
            telemetry.update();
        } else if (!Arrays.equals(beaconColor, beaconRed) & !weAreRed) {
            buttonpusher.setPosition(Range.scale(beacon_left_button_pos, 0, 185, 0, 255));
            telemetry.addData("Pushing left button", "");
            telemetry.update();
        }
        Thread.sleep(50);
        buttonpusher.setPosition(Range.scale(rest_position, 0, 185, 0, 255));
        telemetry.addData("Button Push Successful. Release the triggers NOW.", "");
        telemetry.update();
        Thread.sleep(100);
    }

}
