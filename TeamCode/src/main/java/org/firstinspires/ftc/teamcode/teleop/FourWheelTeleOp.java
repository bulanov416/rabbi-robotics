package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Matan on 10/16/2016.
 */
@TeleOp(name = "Four Wheel BasicDriveTeleOp")
public class FourWheelTeleOp extends LinearOpMode {

    DcMotor l, r; // front motors
    DcMotor lb, rb; // back motors

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1.setJoystickDeadzone((float) 0.05);
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
        }
    }

}
