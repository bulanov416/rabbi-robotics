package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Matan on 11/5/2016.
 */

@Autonomous(name="Encoders PoC", group="Proofs of Concept")
public class EncodersPoC extends LinearOpMode {

    DcMotor motor;

    public void runOpMode() {
        motor = hardwareMap.dcMotor.get("motor");
        drive(1, 100);
    }

    public void drive(double power, int distance) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(distance);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        while (motor.isBusy()) {}
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
