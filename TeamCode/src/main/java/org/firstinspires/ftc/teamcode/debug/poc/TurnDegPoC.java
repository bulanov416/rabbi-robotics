package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.gannacademy.libraries.HardwareRabbi;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Levi on 11/6/2016.
 * Example code for writing Proportional-Derivative control loop to handle smooth turning
 */
@Autonomous(name="TurnDegPD")
public class TurnDegPoC extends LinearOpMode {

    HardwareRabbi robot = new HardwareRabbi();
    BNO055IMU imu;

    public TurnDegPoC() {}

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        turnDeg(90);
    }
    //Turns clockwise
    public void turnDeg(double degrees) throws InterruptedException {
        //Adafruit Config assuming already calibrated - TODO get calibration data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //Initiates IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //Initial orientation
        Orientation orientationI = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
        double bearingYawD = orientationI.firstAngle; //Initial bearing on the yaw axis
        telemetry.addLine("Bearing Current: " + Double.toString(bearingYawD));
        bearingYawD += degrees; //Desired yaw
        double kP = 1; //P co-efficient
        double kD = 0; //D co-efficient
        boolean isStable = false; // this line of code is designed to instill incredible confidence in the IMU
        while(!isStable) {
            //Current Orientation
            Orientation orientationC = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.YXZ);
            double bearingYawC = orientationC.firstAngle; //Initial bearing on the yaw axis
            long time = System.currentTimeMillis()+20; //Curent time
            double errorValue = (bearingYawD - bearingYawC)/360; //Difference between current and desired value
            double errorDerivative = imu.getAngularVelocity().firstAngleRate/50; //Angular velocity in deg/tick
            double uT = (kP * errorValue) + (kD * errorDerivative);
            if (uT > 1) uT = 1;
            if (uT < -1) uT = -1;
            double lT = uT; robot.setLeftPower(lT); //Sets left power
            double rT = -1*uT; robot.setRightPower(rT); //Sets right power
            if (!(System.currentTimeMillis() > time)) {
                Thread.sleep(time-System.currentTimeMillis(), 0);
            }
            while (Math.abs(errorValue) <= 1 && Math.abs(errorDerivative) <= 3) {
                robot.stopDriving();
                orientationC = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.YXZ);
                bearingYawC = orientationC.firstAngle;
                errorValue = bearingYawD - bearingYawC;
                Thread.sleep(20);
                if (System.currentTimeMillis() > time + 80) {
                    robot.stopDriving();
                    isStable = true;
                    break;
                }
            }
        }
    }
}
