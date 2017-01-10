package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.gannacademy.libraries.HardwareRabbi;


/**
 * Created by Matan on 11/2/2016.
 */

@TeleOp(name = "Catapult")
public class Catapult extends LinearOpMode {

    DcMotor catapult;
    DcMotor l, r; // front motors
    DcMotor lb, rb; // back motors

    public void runOpMode() {
        setupRobot();
        while (opModeIsActive()) {
            l  = hardwareMap.dcMotor.get("l");
            r  = hardwareMap.dcMotor.get("r");
            lb = hardwareMap.dcMotor.get("lb");
            rb = hardwareMap.dcMotor.get("rb");
            float left_power = Range.clip(gamepad1.left_stick_y, -1, 1);
            float right_power = Range.clip(gamepad1.left_stick_y, -1, 1);
            l.setPower(left_power);
            r.setPower(right_power);
            lb.setPower(left_power);
            rb.setPower(right_power);

            if (gamepad1.dpad_up) {
                catapult.setPower(1);
            } else {
                catapult.setPower(0);
            }
        }
    }

    public void setupRobot() {
        catapult = hardwareMap.dcMotor.get("catapult");
    }

}
