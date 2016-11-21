package org.firstinspires.ftc.teamcode.archive.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.gannacademy.libraries.HardwareRabbi;


/**
 * Created by Matan on 11/5/2016.
 */

@Autonomous(name="Encoders PoC", group="Proofs of Concept")
// @Disabled
public class EncodersPoC extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();

    public void runOpMode() {
        driveWithEncoderPoC(1, 100);
    }

    public void driveWithEncoderPoC(double power, int distance) {
        robot.l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.l.setTargetPosition(distance);
        robot.l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.l.setPower(power);
        while (robot.l.isBusy()) {}
        robot.l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
