package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Levi on 11/5/2016.
 * Implements a PID for line following, intended for us as
 * final beacon approach mechanism. Use guidance through
 * image recognition to identify beacon, then simply run program
 * when close enough. Use forward mounted ultrasonic
 * to stop at appropriate distance.
 */
@TeleOp(name = "LineFollower")
public class LineFollowPoC extends LinearOpMode {

    HardwareRabbi hardwareRabbi = new HardwareRabbi();

    public LineFollowPoC() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Initial implementation of PID for final beacon alignment, PoC
        double threshold = 0.38; //Set properly with actual value
        hardwareRabbi.light.enableLed(true);
        double kP = 2.5; //P co-efficient
        double kI = 0; //I co-efficient
        double kD = 0; //D co-efficient
        double errorIntegral = 0; //Integral for running sum, iterative algorithm
        double errorDerivative = 0; //Change in error from previous error value
        double errorValue = 0; //Actual error from threshold
        double pT = 0.5; //Desired BALANCED motor power.
        //Loop, iterates 50 times per second while distance is greater than UDS level 8
        while (hardwareRabbi.uds.getUltrasonicLevel() >= 4) {
            long time = System.currentTimeMillis()+20;
            errorValue = threshold - hardwareRabbi.light.getLightDetected(); //Error
            errorDerivative = errorValue - errorDerivative; //Change in Error
            errorIntegral += errorValue; //Running sum of error (integral)
            double uT = (kP * errorValue) + (kI*errorIntegral) + (kD*errorDerivative); //PID equation
            if (uT > 1) uT = 1; if (uT < -1) uT = -1;
            double lT = pT + uT; //Left motor power
            double rT = pT - uT; //Right motor power
            hardwareRabbi.setLeftPower(lT); hardwareRabbi.setRightPower(rT); //Sets power for tick
            if (!(System.currentTimeMillis() > time)) {
                Thread.sleep(time-System.currentTimeMillis(), 0);
            }
        }
        hardwareRabbi.stopDriving();
        //Insert button push method here

        //Drive forwards untill ultrasonic reads < 1 here

    }
}
