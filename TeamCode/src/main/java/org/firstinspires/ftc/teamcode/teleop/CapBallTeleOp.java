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
            if (gamepad1.dpad_up) {
                robot.stopDriving();
                telemetry.addData("Button Pushed", "DPAD_UP");
                telemetry.update();
                while (gamepad1.dpad_up) {liftBall();}
            }
            else if (gamepad1.dpad_down) {
                robot.stopDriving();
                telemetry.addData("Button Pushed", "DPAD_DOWN");
                telemetry.update();
                while (gamepad1.dpad_down) {lowerBall();}
            }
            robot.lift.setPower(0);
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

    public void liftBall() {
        telemetry.addData("Cap Ball Motor", "LIFTING");
        robot.lift.setPower(1);
    }

    public void lowerBall() {
        telemetry.addData("Cap Ball Motor", "LOWERING");       
        robot.lift.setPower(-1);
    }
}
