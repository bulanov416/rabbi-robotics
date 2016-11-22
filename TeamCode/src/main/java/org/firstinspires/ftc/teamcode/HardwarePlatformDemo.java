package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.gannacademy.libraries.DriveController;
import org.gannacademy.libraries.HardwarePlatform;
import org.gannacademy.libraries.SensorController;

/**
 * Created by Nathan on 11/21/2016.
 */
@Autonomous(name = "HardwarePlatform Demo")
// @Disabled
public class HardwarePlatformDemo extends LinearOpMode {

    HardwarePlatform robot;

    public HardwarePlatformDemo() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwarePlatform();
        robot.init(hardwareMap, telemetry);
        DriveController driveController = new DriveController(robot);
        SensorController sensors = new SensorController(robot);

        waitForStart();

        while (opModeIsActive()) {
            driveController.driveSeconds(2000, 1);
            Thread.sleep(1000);
            driveController.driveSeconds(1900, -1);
            Thread.sleep(1000);
        }
        stop();
    }
}
