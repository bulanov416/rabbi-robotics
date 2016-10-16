package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Matan on 10/16/2016.
 */
public class FourWheelDrive extends LinearOpMode {
    DcMotor l, r; // front motors
    DcMotor lb, rb; // back motors

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
        } else if (negative_power < 0) {
            return (float) negative_power;
        } else {
            return 0;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        while (true) {
            l  = hardwareMap.dcMotor.get("l");
            r  = hardwareMap.dcMotor.get("r");
            lb = hardwareMap.dcMotor.get("lb");
            rb = hardwareMap.dcMotor.get("br");

            float power = scale_motor_power((double) gamepad1.right_stick_y, 0.05);
            float direction = scale_motor_power((double) gamepad1.left_stick_x, 0.05);

            if (direction > 0) { // right
                l.setPower(power);
                lb.setPower(power);
            } else if (direction < 0){ // left
                r.setPower(power);
                rb.setPower(power);
            } else { // strait
                r.setPower(power);
                l.setPower(power);
                rb.setPower(power);
                lb.setPower(power);
            }

        }
    }
}
