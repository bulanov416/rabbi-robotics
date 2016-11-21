package org.firstinspires.ftc.teamcode.archive.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Levi on 10/30/2016
 */
@TeleOp(name = "ActiveCalAutoPoc")
public class ActiveCalAutoPoc extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    UltrasonicSensor us;

    public ActiveCalAutoPoc() {

    }

    @Override
    public void runOpMode() {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");

        us = hardwareMap.ultrasonicSensor.get("us");

        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        long timeFinal = System.currentTimeMillis() + 4000;
        double distanceInital = us.getUltrasonicLevel();
        while (System.currentTimeMillis() < timeFinal) {
            r.setPower(1);
            l.setPower(1);
            rb.setPower(1);
            lb.setPower(1);
        }
        r.setPower(0);
        l.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        double distanceFinal = us.getUltrasonicLevel();
        float speed = (float) (distanceFinal - distanceInital)/4;
    }
}