package org.firstinspires.ftc.teamcode.archive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;

/**
 * Created by Nathan on 10/16/2016.
 */
public class ButtonPusherTeleOp extends LinearOpMode {

    DcMotor l, r, lb, rb;
    Servo buttonpusher;
    ColorSensor colorSensor;
    int beacon_left_button_pos, beacon_right_button_pos, rest_position;
    int[] beaconRed = new int[4];

    public ButtonPusherTeleOp() {}

    public void runOpMode() throws InterruptedException {
        gamepad1.setJoystickDeadzone((float) 0.05);

        l  = hardwareMap.dcMotor.get("l");
        r  = hardwareMap.dcMotor.get("r");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("br");
        buttonpusher = hardwareMap.servo.get("buttonpusher servo");
        colorSensor = hardwareMap.colorSensor.get("buttonpusher beaconColor");


        waitForStart();
        while (true) {
            float left_power = Range.clip(gamepad1.left_stick_y, -1, 1);
            float right_power = Range.clip(gamepad1.right_stick_x, -1, 1);

            l.setPower(left_power);
            lb.setPower(left_power);
            r.setPower(right_power);
            rb.setPower(right_power);
            
            if (gamepad1.right_trigger > 0 & gamepad1.left_trigger > 0) {
                int[] beaconColor = new int[4];
                boolean weAreRed;
                beaconColor[0] = colorSensor.red();
                beaconColor[1] = colorSensor.green();
                beaconColor[2] = colorSensor.blue();
                beaconColor[3] = colorSensor.alpha();
                telemetry.addData("Select a team:", "L:Right, B:Left");
                telemetry.update();

                do {
                    if (gamepad1.dpad_right) {
                        weAreRed = false;
                        telemetry.addData("We are: ", "BLUE");
                    } else {
                        weAreRed = true;
                        telemetry.addData("We are:", "RED");
                    }
                    telemetry.update();
                } while (!gamepad1.dpad_left || !gamepad1.dpad_right);

                if (Arrays.equals(beaconColor, beaconRed) & weAreRed) {
                    buttonpusher.setPosition(beacon_left_button_pos);
                    telemetry.addData("Pushing left button", "");
                    telemetry.update();
                } else if (Arrays.equals(beaconColor, beaconRed) & !weAreRed){
                    buttonpusher.setPosition(beacon_right_button_pos);
                    telemetry.addData("Pushing right button", "");
                    telemetry.update();
                } else if (!Arrays.equals(beaconColor, beaconRed) & weAreRed) {
                    buttonpusher.setPosition(beacon_right_button_pos);
                    telemetry.addData("Pushing right button", "");
                    telemetry.update();
                } else if (!Arrays.equals(beaconColor, beaconRed) & !weAreRed) {
                    buttonpusher.setPosition(beacon_left_button_pos);
                    telemetry.addData("Pushing left button", "");
                    telemetry.update();
                }
                Thread.sleep(50);
                buttonpusher.setPosition(rest_position);
                telemetry.addData("Button Push Successful. Release the triggers NOW.", "");
                telemetry.update();
                Thread.sleep(100);
            }
        }
    }

    /* float scale_motor_power_redundant(double p_power, double deadzone) { // DcMotor.setPower needs a float
        // Simpler method of controlling the motor range
        p_power = Range.clip(p_power, -1, 1);
        double negative_power = 0, positive_power = 0;
        // differentiate between negative and positive power - required to implement deadzones
        if (p_power > 0) {
            positive_power = Range.clip(p_power, deadzone, 1); // implement the deadzone
            positive_power = Range.scale(positive_power, deadzone, 1, 0, 1); // and bring it back to 0<n<1
        } else {
            negative_power = Range.clip(p_power, -deadzone, -1); // implement the deadzone
            negative_power = Range.scale(negative_power, -deadzone, -1, 0, -1); // and bring it back to 0>n>-1
        }
        // return the final power
        if (positive_power > 0) {
            return (float) positive_power;
        }
        else if (negative_power < 0) {
            return (float) negative_power;
        }
        else {
            return 0;
        }
    } */

}
