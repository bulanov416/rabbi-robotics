package org.firstinspires.ftc.teamcode.archive.debug.poc;

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
        robot.init(hardwareMap, telemetry);

        waitForStart();

        robot.driveCentimeters(30, 1);
        Thread.sleep(1000);
        robot.driveCentimeters(15, -1);
        robot.driveCentimeters(5, 1);
        Thread.sleep(500);
        robot.driveCentimeters(20, -1);
        telemetry.addData("Full Power test", "COMPLETE");
        telemetry.update();
        Thread.sleep(1000);
        robot.driveCentimeters(30, 0.65);
        Thread.sleep(1000);
        robot.driveCentimeters(15, -0.65);
        robot.driveCentimeters(5, 0.65);
        Thread.sleep(500);
        robot.driveCentimeters(20, -0.65);
        telemetry.addData("65% Power test", "COMPLETE");
        telemetry.update();
        Thread.sleep(1000);
    }
}
