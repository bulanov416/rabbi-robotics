package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 11/10/2016.
 */
@TeleOp(name = "Cap Ball TeleOp")
// @Disabled
public class CapBallTeleOp extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();

    double right_power;
    double left_power;

    public CapBallTeleOp() {}

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drive();
            if (gamepad1.a) {
                robot.stopDriving();
                liftBall();
            }
            if (gamepad1.b) {
                robot.stopDriving();
                lowerBall();
            }
            robot.capBallLift.setPower(0);
        }
    }

    public void drive() {
        left_power = gamepad1.left_stick_y;
        right_power = gamepad1.right_stick_y;
        Range.clip(left_power, -1, 1);
        Range.clip(right_power, -1, 1);
        robot.l.setPower(left_power);
        robot.lb.setPower(left_power);
        robot.r.setPower(right_power);
        robot.rb.setPower(right_power);
    }

    public void liftBall() {
        robot.capBallLift.setPower(0.4);
    }

    public void lowerBall() {
        robot.capBallLift.setPower(-0.4);
    }
}
