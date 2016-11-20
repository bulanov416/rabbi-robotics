package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 11/17/2016.
 */
@TeleOp(name="Simplified 4WD TeleOp")
public class SimpleDriveTeleOp extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();

    double left_power;
    double right_power;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drive();
        }
        stop();
    }

    public void drive() {
        left_power = gamepad1.left_stick_y;
        right_power = gamepad1.right_stick_y;
        Range.clip(left_power, -1, 1);
        Range.clip(right_power, -1, 1);
        robot.l.setPower(left_power);
        robot.lb.setPower(left_power);
        robot.r.setPower(right_power);
        robot.rb.setPower(-right_power);
    }
}
