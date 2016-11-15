package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.gannacademy.libraries.HardwareRabbi;

@Autonomous(name="Working Beacon Pusher")
public class BeaconAlignAuto extends LinearOpMode {

    private HardwareRabbi robot = new HardwareRabbi();

    public BeaconAlignAuto() {}

    public void runOpMode() throws InterruptedException {
        boolean isRed = true;
        OpticalDistanceSensor ods = hardwareMap.opticalDistanceSensor.get("ods");
        robot.init(hardwareMap, telemetry);
        waitForStart();
        robot.drive(0.65);
        while (ods.getRawLightDetected() < 0.04) {
            Thread.sleep(50);
        }
        robot.driveCentimeters(19.5, 0.65);
        if (isRed) robot.setLeftPower(0.6);
        else robot.setRightPower(0.6);
        while (ods.getRawLightDetected() < 0.4) {
            Thread.sleep(50);
        }
        robot.stopDriving();
    }
}
