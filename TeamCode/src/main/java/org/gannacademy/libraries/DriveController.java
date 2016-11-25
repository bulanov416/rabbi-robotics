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
        robot.telemetry.addLine("Using DriveController v1.0.1-161126");
        robot.telemetry.update();
    }

    /* *****************
     * DRIVING METHODS *
     *******************/
    public void drive(double power) {
        this.setLeftPower(power);
        this.setRightPower(power);
    }

    public void driveSeconds(double seconds, double power) throws InterruptedException {
        drive(power);
        Thread.sleep((long) (seconds * 1000));
        stopDriving();
    }

    public void stopDriving() {
        this.drive(0);
    }

    public void turnLeft(double power) {
        this.setLeftPower(power);
        this.setRightPower(-power);
    }

    public void turnRight(double power) {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setLeftPower(-power);
        this.setRightPower(power);
    }

    public void setLeftPower(double power) {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.l.setPower(power);
        robot.lb.setPower(power);
    }

    public void setRightPower(double power) {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.r.setPower(power);
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
