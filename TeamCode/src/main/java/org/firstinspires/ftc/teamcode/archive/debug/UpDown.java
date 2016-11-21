package org.firstinspires.ftc.teamcode.archive.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Matan on 10/23/2016.
 */
@TeleOp(name = "Up Down")
public class UpDown extends LinearOpMode {
    DcMotor motor;
    public void runOpMode() throws InterruptedException {
        gamepad1.setJoystickDeadzone((float) 0.05);
        motor = hardwareMap.dcMotor.get("motor");

        // in Java, while (true) can only exit by throwing an exception. This is not OK.
        // Instead, we use while (opModeIsActive()), which exits when the OpMode is closed.
        while (opModeIsActive()) {
            double power = Range.clip(gamepad1.left_stick_y, -1, 1);
            motor.setPower(power);
        }
    }
}
