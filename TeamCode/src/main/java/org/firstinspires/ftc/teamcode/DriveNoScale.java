package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by alexbulanov on 9/28/16.
 */

@TeleOp(name = "Drive without Scaling")
public class DriveNoScale extends LinearOpMode {
    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor lift;
    Servo button_left;
    Servo button_right;

    float motorPowerScale = 1;


    public DriveNoScale() {}

    double scale_motor_power(double p_power)  {
        Range.clip(p_power, -1, 1);
        return p_power*motorPowerScale;
    }

    @Override
    public void runOpMode() {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        lift = hardwareMap.dcMotor.get("lift");
        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        while (opModeIsActive()) {
            float gp1_left_stick_y = gamepad1.left_stick_y;
            float left_drive_power
                    = (float) scale_motor_power(gp1_left_stick_y);

            float gp1_right_stick_y = gamepad1.right_stick_y;
            float right_drive_power
                    = (float) scale_motor_power(gp1_right_stick_y);

            r.setPower(right_drive_power);
            l.setPower(left_drive_power);
            rb.setPower(right_drive_power);
            lb.setPower(left_drive_power);

            if (gamepad1.dpad_up) {
                lift.setPower(1*motorPowerScale);
            }
            if (gamepad1.dpad_down) {
                lift.setPower(-1*motorPowerScale);
            }
            if (gamepad1.y) {
                lift.setPower(0);
            }
            if (gamepad1.left_bumper) {
                button_left.setPosition(0.05);
            }
            if (gamepad1.right_bumper) {
                button_right.setPosition(0.95);
            }
            if (gamepad1.b) {
                button_left.setPosition(0.9);
                button_right.setPosition(0.1);
            }

            if (gamepad1.left_stick_button && !gamepad1.right_stick_button) {
                motorPowerScale = 0.5f;
            }
            else  if (gamepad1.right_stick_button && !gamepad1.left_stick_button) {
                motorPowerScale = 1.0f;
            }



        }
        l.setPower(0);
        l.close();
        r.setPower(0);
        r.close();
        lb.setPower(0);
        lb.close();
        rb.setPower(0);
        rb.close();
        lift.setPower(0);
        lift.close();
        button_left.close();
        button_right.close();
        stop();
    }
}