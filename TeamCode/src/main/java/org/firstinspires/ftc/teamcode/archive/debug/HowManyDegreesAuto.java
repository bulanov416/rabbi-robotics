package org.firstinspires.ftc.teamcode.archive.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 11/7/16.
 */
public class HowManyDegreesAuto extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();

    public HowManyDegreesAuto() {}

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        robot.turnLeft(1, 1);
    }
}
