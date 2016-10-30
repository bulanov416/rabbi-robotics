package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 10/30/2016.
 */
@Autonomous(name="driveCentimeters() PoC", group="Proofs of Concept")
public class DriveCentimetersPoC extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();

    public DriveCentimetersPoC() {}

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.driveCentimeters(10, 100);
        robot.stopDriving();
        Thread.sleep(1000);
        robot.driveCentimeters(-10, 100);
        stop();
    }


}
