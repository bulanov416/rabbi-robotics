package org.gannacademy.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Nathan on 11/21/2016.
 */
public class DriveController {

    HardwarePlatform robot;

    public DriveController(HardwarePlatform hwPlatform) {
        robot = hwPlatform;
        robot.telemetry.addLine("Using DriveController v1.0.0-161126");
        robot.telemetry.update();
    }

    /* *****************
     * DRIVING METHODS *
     *******************/
    public void drive(double power) {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.l.setPower(power);
        robot.r.setPower(power);
        robot.lb.setPower(power);
        robot.rb.setPower(power);
    }

    public void driveSeconds(long milliseconds, int power) throws InterruptedException {
        drive(power);
        Thread.sleep(milliseconds);
        stopDriving();
    }

    public void stopDriving() {
        robot.l.setPower(0);
        robot.r.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }

    public void turnLeft(double power) {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.l.setPower(power);
        robot.r.setPower(-power);
        robot.lb.setPower(power);
        robot.rb.setPower(-power);
    }

    public void turnRight(double power) {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.l.setPower(-power);
        robot.r.setPower(power);
        robot.lb.setPower(-power);
        robot.rb.setPower(power);
    }
    /* *********************
     * MOTOR OPTION METHODS*
     ***********************/
    public void setDriveMode(DcMotor.RunMode runMode) {
        robot.l.setMode(runMode);
        robot.r.setMode(runMode);
        robot.lb.setMode(runMode);
        robot.rb.setMode(runMode);
    }
}
