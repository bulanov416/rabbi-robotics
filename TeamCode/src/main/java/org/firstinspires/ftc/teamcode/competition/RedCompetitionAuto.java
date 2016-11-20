package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ThreadPool;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 11/20/2016.
 */
@Autonomous(name = "Red Automomous", group = "Competition OpModes")
// @Disabled
public class RedCompetitionAuto extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();
    private int beacon_left = 135, beacon_right = 45, rest_position = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        //scaleServoValues();
        waitForStart();
        //~46 ticks per centimeter
        //while (opModeIsActive()) {
            // or robot.driveCentimeters(50, 1);
            robot.driveTicks(1, 2285);
            Thread.sleep(500);
            robot.driveCentimeters(45, -1);
            // or robot.driveTicks(-1, 2070);
            /*robot.turnLeftTicks(1, 2450);
            // or robot.turnRightTicks(1, 100);
            robot.drive(0.75);
            while (robot.eods.getRawLightDetected() > 0.04) {Thread.sleep(50);}
            robot.stopDriving();
            robot.driveCentimeters(0.75, 19.5);
            // or robot.
            robot.stopDriving();
            robot.setLeftPower(0.75);
            while (robot.eods.getRawLightDetected() > 0.04) {Thread.sleep(50);}
            robot.stopDriving();
            robot.button_pusher.setPosition(rest_position);*/
            //while (opModeIsActive()) {Thread.sleep(500);};
        //}
        while (opModeIsActive()) {
            Thread.sleep(10000);
            robot.l.setPower(0.75);
            robot.r.setPower(0.75);
            robot.lb.setPower(0.75);
            robot.lb.setPower(0.75);
            Thread.sleep(5000);
            robot.l.setPower(0);
            robot.r.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);
            Thread.sleep(100);
            robot.l.setPower(-0.75);
            robot.r.setPower(-0.75);
            robot.lb.setPower(-0.75);
            robot.lb.setPower(-0.75);
            Thread.sleep(2000);
            robot.l.setPower(0);
            robot.r.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);
        }
        stop();
    }

    public void scaleServoValues() {
        Range.scale(beacon_left, 0, 180, 0, 255);
        Range.scale(beacon_right, 0, 180, 0, 255);
        Range.scale(rest_position, 0, 180, 0, 255);
        telemetry.addData("Beacon Values", "SET");
        telemetry.update();
    }
}
