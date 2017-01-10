package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by alexbulanov on 11/20/2016.
 */
@Autonomous(name = "Red Automomous", group = "Competition OpModes")
public class RedCompetitionAuto extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;

    @Override
    public void runOpMode() throws InterruptedException {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");

        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        telemetry.addLine("Motor Nulls:" + l != null ? "false" : "true");
        l.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        Thread.sleep(9000);
        l.setPower(0.75);
        r.setPower(0.75);
        lb.setPower(0.75);
        rb.setPower(0.75);
        Thread.sleep(2325);
        l.setPower(0);
        r.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        Thread.sleep(500);
        stop();
    }
}
