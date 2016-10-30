package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Matan on 10/16/2016.
 */
@TeleOp(name="Fly3TeleOp")
public class Fly3TeleOp extends LinearOpMode {
    // declare motors and servos
    DcMotor motor1, motor2, motor3, motor4, motor5;
    Servo servo1, servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        double power = 0;

        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor5 = hardwareMap.dcMotor.get("motor5");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

        // increment / decrement power
        for (int i = 0; i < 12; i++) {
            // if we're halfway through, start decrementing
            if (i > 6) {
                power -= 10;
            } else {
                power += 10;
            }
            setPower(power);
            Thread.sleep(5000);
        }
    }
    void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
        motor5.setPower(power);
        servo1.setPosition(power);
        servo2.setPosition(power);
    }
}
