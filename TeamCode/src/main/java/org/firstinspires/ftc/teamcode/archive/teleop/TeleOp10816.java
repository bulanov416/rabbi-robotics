package org.firstinspires.ftc.teamcode.archive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
A small note about the phones:
The one with the BLUE wallpaper is the Robot Controller.
The one with the ORANGE wallpaper is the Driver Station.
 */

@TeleOp(name = "TeleOp 10/8/16")
public class TeleOp10816 extends LinearOpMode {

    DcMotor l, r; // always shorten these!
    DcMotor lb, rb;
    DcMotor fly1, fly2;
    Servo bring;
    Servo clasp1, clasp2;


    public TeleOp10816() {

    }

    double scale_motor_power_legacy (double p_power)  //Scales joystick value to output appropriate motor power
    {                                          //Use like "scale_motor_power(gamepad1.left_stick_x)"

        double l_scale = 0.0; // Assume no scaling.

        double l_power = Range.clip (p_power, -1, 1); // Ensure the values are legal.

        double[] l_array =
                { 0.00, 0.05, 0.09, 0.10, 0.12
                        , 0.15, 0.18, 0.24, 0.30, 0.36
                        , 0.43, 0.50, 0.60, 0.72, 0.85
                        , 1.00, 1.00
                };

        int l_index = (int) (l_power * 16.0); // Get the corresponding index for the specified argument/parameter.

        if (l_index < 0) {
            l_index = -l_index;
        }
        else if (l_index > 16) {
            l_index = 16;
        }

        if (l_power < 0) {
            l_scale = -l_array[l_index];
        }
        else {
            l_scale = l_array[l_index];
        }

        return l_scale;

    }

    float scale_motor_power(double p_power, double deadzone) { // DcMotor.setPower needs a float
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
    }

    @Override public void runOpMode () {
        while(true) {

            l = hardwareMap.dcMotor.get("l");
            r = hardwareMap.dcMotor.get("r");
            rb = hardwareMap.dcMotor.get("rb");
            lb = hardwareMap.dcMotor.get("lb");
            fly1 = hardwareMap.dcMotor.get("fly1");
            fly2 = hardwareMap.dcMotor.get("fly2");
            bring = hardwareMap.servo.get("bring");
            clasp2 = hardwareMap.servo.get("clasp2");
            clasp1 = hardwareMap.servo.get("clasp1");

            fly2.setDirection(DcMotor.Direction.REVERSE);
            r.setDirection(DcMotor.Direction.REVERSE);
            rb.setDirection(DcMotor.Direction.REVERSE);

            float l_gp1_left_stick_y = gamepad1.left_stick_y;
            float l_left_drive_power
                    = scale_motor_power((double) l_gp1_left_stick_y, 0.05);

            float l_gp1_right_stick_y = gamepad1.right_stick_y;
            float l_right_drive_power
                    = scale_motor_power((double) l_gp1_right_stick_y, 0.05);

            r.setPower(l_right_drive_power);
            l.setPower(l_left_drive_power);
            rb.setPower(l_right_drive_power);
            lb.setPower(l_left_drive_power);

            while (gamepad1.dpad_up) {

                fly1.setPower(1);
                fly2.setPower(1);

            }

            //When button a is pressed, the ball function is executed.
            if (gamepad2.dpad_down) {

                bring.setPosition(1);

            }

            if (gamepad2.dpad_up) {

                clasp2.setPosition(0);

            }

            if (gamepad2.dpad_left) {

                clasp2.setPosition(1);
                clasp1.setPosition(1);

            }

            if (gamepad2.dpad_right) {

                clasp2.setPosition(0);
                clasp1.setPosition(0);

            }
            if (!gamepad1.dpad_down) {
                fly1.setPower(0);
                fly2.setPower(0);
            }
            fly1.setPower(0);
            fly2.setPower(0);
        }
        /*clasp2.setPosition(1);
        sleep(1000);
        bring.setPosition(0.9);
        sleep(2000);
        clasp1.setPosition(0.5);
        clasp2.setPosition(0.5);*/
    }

        //This is the function to pick up balls and bring them to the launcher
       /* public void ball() {

        clasp1.setPosition(1);
        clasp2.setPosition(1);
        sleep(1000);
        bring.setPosition(0.9);
        sleep(2000);
        clasp1.setPosition(0.5);
        clasp2.setPosition(0.5);

    }
*/
    //This function should be used for sleep because it is setup to throw InterruptedException.

}


