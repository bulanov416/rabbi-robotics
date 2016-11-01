package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Levi on 10/30/2016
 */
@TeleOp(name = "VuforiaPoc")
public class VuforiaPoC extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;

    public VuforiaPoC() {

    }

    @Override
    public void runOpMode() {

        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        while (true) {

            r.setDirection(DcMotor.Direction.REVERSE);
            rb.setDirection(DcMotor.Direction.REVERSE);



        }
    }
}