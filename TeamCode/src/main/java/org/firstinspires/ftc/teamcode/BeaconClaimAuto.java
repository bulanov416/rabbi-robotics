package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.gannacademy.libraries.ButtonPusherFunctions;
import org.gannacademy.libraries.DriveController;
import org.gannacademy.libraries.HardwarePlatform;
import org.gannacademy.libraries.HardwareRabbi;
import org.gannacademy.libraries.SensorController;

/**
 * Created by Levi on 11/5/2016.
 * Implements a PID for line following, intended for us as
 * final beacon approach mechanism. Use guidance through
 * image recognition to identify beacon, then simply run program
 * when close enough. Use forward mounted ultrasonic
 * to stop at appropriate distance.
 */
@Autonomous(name = "BeaconPushStartAuto")
public class BeaconClaimAuto extends LinearOpMode {

    HardwarePlatform robot;
    DriveController driveController;
    SensorController sensors;
    ButtonPusherFunctions pusher;

    public BeaconClaimAuto() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwarePlatform();
        robot.init(hardwareMap, telemetry);
        driveController = new DriveController(robot);
        sensors = new SensorController(robot);
        pusher = new ButtonPusherFunctions(robot);

        waitForStart();;

        //Drives Forward until line spotted
        driveController.drive(0.65);
        while (!pusher.robotIsOnWhite()) {
            Thread.sleep(50);
        }
        //Replace with better drive method when developed;
        driveController.driveSeconds(0.1, 0.3);
        if (HardwarePlatform.isRed) driveController.turnLeft(0.4);
        else driveController.turnRight(0.4);
        while (!pusher.robotIsOnWhite()) {
            Thread.sleep(20);
        }
        driveController.stopDriving();
        Thread.sleep(100);
        //Follows line untill beacon detected
        this.FollowLine();
        //Sets pusher position
        pusher.setPusherPosition();
        while (!pusher.isBeaconPressed()) {
            driveController.drive(0.1);
            Thread.sleep(40);
        }
        //Add code to drive back, turn 90 degrees, then rine and repeat
    }

    public void FollowLine() throws InterruptedException{
        double threshold = 0.38; //Set properly with actual value
        double kP = 2.5; //P co-efficient
        double kI = 0; //I co-efficient
        double kD = 0; //D co-efficient
        double errorIntegral = 0; //Integral for running sum, iterative algorithm
        double errorDerivative = 0; //Change in error from previous error value
        double errorValue = 0; //Actual error from threshold
        double pT = 0.5; //Desired BALANCED motor power.
        //Loop, iterates 50 times per second while distance is greater than button pusher poles
        while (!robot.beaconTouch.isPressed()) {
            long time = System.currentTimeMillis()+20;
            errorValue = threshold - robot.eods.getRawLightDetected(); //Error
            errorDerivative = errorValue - errorDerivative; //Change in Error
            errorIntegral += errorValue; //Running sum of error (integral)
            double uT = (kP * errorValue) + (kI*errorIntegral) + (kD*errorDerivative); //PID equation
            if (uT > 0.5) uT = 0.5; if (uT < -0.5) uT = -0.5;
            double lT = pT + uT; //Left motor power
            double rT = pT - uT; //Right motor power
            driveController.setLeftPower(lT); driveController.setRightPower(rT); //Sets power for tick
            if (!(System.currentTimeMillis() > time)) {
                Thread.sleep(time-System.currentTimeMillis(), 0);
            }
        }
        driveController.stopDriving();
    }
}
