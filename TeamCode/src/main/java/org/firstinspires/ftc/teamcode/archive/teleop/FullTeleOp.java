package org.firstinspires.ftc.teamcode.archive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Matan on 11/14/2016.
 * Contains drive, capball, and beacon push
 */

@TeleOp(name = "Full TeleOp")
public class FullTeleOp extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();

    private final int beacon_left_button_pos = 135,
                      beacon_right_button_pos = 45,
                      rest_position = 90;

    public void runOpMode() {

        setup();

        while (opModeIsActive()) {

            float power_r = Range.clip(gamepad1.right_stick_x, 1, -1);
            float power_l = Range.clip(gamepad1.left_stick_y, 1, -1);

            setPowerRight(power_r);
            setPowerLeft(power_l);

            if (gamepad2.left_bumper) {
                robot.lift.setPower(-1);
            }  else if (gamepad2.right_bumper){
                robot.lift.setPower(1);
            }  else {
                robot.lift.setPower(0);
            }

            if (gamepad2.dpad_left) {
                robot.button_pusher.setPosition(beacon_left_button_pos);
            } else if (gamepad2.dpad_right) {
                robot.button_pusher.setPosition(beacon_right_button_pos);
            } else if (gamepad2.dpad_down){
                robot.button_pusher.setPosition(rest_position);
            }

        }


    }


    public void setPowerLeft(double power) {
        robot.r.setPower(power);
        robot.rb.setPower(power);
    }
    public void setPowerRight(double power) {
        robot.l.setPower(power);
        robot.rb.setPower(power);
    }

    public void setup() {
        robot.init(hardwareMap, telemetry);
        gamepad1.setJoystickDeadzone((float) 0.05);
    }

}
