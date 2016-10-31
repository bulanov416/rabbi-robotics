package org.gannacademy.libraries;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.TelemetryInternal;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for our robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor: Left  drive motor:        "l"
 * Motor: Right drive motor:        "r"
 * Motor: Left back drive motor:    "lb"
 * Motor: Right Back BasicDriveTeleOp Motor:   "rb"
 * Servo: Button Pusher:            "buttonPushServo"
 *
 * More motors can be added as they are implemented
 */
public class HardwareRabbi
{
    /* Public OpMode members. */
    public DcMotor l;
    public DcMotor r;
    public DcMotor lb;
    public DcMotor rb;
    public Servo   buttonPushServo;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();

    /* Enum ButtonPushPositions {
        LEFT_BUTTON, REST_POSITION, RIGHT_BUTTON;
    }; */

    /* Constructor */
    public HardwareRabbi(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap; // uses the HardwareMap from the OpMode

        // Define and Initialize Motors
        l  = hwMap.dcMotor.get("l");
        lb = hwMap.dcMotor.get("lb");
        r  = hwMap.dcMotor.get("r");
        rb = hwMap.dcMotor.get("rb");

        l .setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        r .setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        // Initially stop all motors
        stopDriving();
        // Make sure the motors are not expecting encoders
        l .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the servo
        buttonPushServo = hwMap.servo.get("buttonPushServo");
        // Set it's position
        buttonPushServo.setPosition(90); // replace with vars
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /***
     * Drives the robot for a certain distance, defined by centimeters. To figure out how long to
     * run the motors, a conversion factor between centimeters and seconds is applied. Negative
     * distances move the robot backwards.
     *
     * @param distance  the distance to drive in Centimeters.
     * @throws InterruptedException
     */
    public void driveCentimeters (double distance, double powerPercentage) throws InterruptedException {
        // drives a distance in cm based on a conversion factor from seconds
        double conversionFactor = 30; // this needs to be filled
        double powerConversionFactor = powerPercentage / 100; // for converting power
        double distanceInSeconds = distance / conversionFactor;
        driveSeconds(100, distanceInSeconds * powerConversionFactor);
        // in the future, we should set up a way to drive at lower powers
    }

    /***
     * Drives the robot for a certain amount of time, at a certain power. Time is in Seconds, power is
     * between 1 and -1. Negative power drives the robot backwards.
     *
     * @param power  the power to run the motors at (between -1 and 1)
     * @param time  the amount of seconds to run the motors for (negative number)
     * @throws InterruptedException
     */
    public void driveSeconds(double power, double time) throws InterruptedException {
        if (time < 0) {
            System.exit(1); // crash with exit code 1, which means an error occured
        }
        r.setPower(power);
        rb.setPower(power);
        l.setPower(power);
        lb.setPower(power);
        Thread.sleep((long) time * 100);
        stopDriving();
    }

    /***
     * Starts driving at a certain power. stopDriving must be called manually afterwards.
     *
     * @param power the power to run the motor at (between -1 and 1)
     */
    public void startDriving(double power) {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(power);
        lb.setPower(power);
    }

    public void turnLeft(double power, double time) throws InterruptedException {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(-power);
        lb.setPower(-power);
        Thread.sleep((long) (time * 1000));
        stopDriving();
    }

    public void turnRight(double power, double time) throws InterruptedException {
        r.setPower(-power);
        rb.setPower(-power);
        l.setPower(power);
        lb.setPower(power);
        Thread.sleep((long) (time * 1000));
        stopDriving();
    }

    public void stopDriving() {
        r.setPower(0);
        rb.setPower(0);
        l.setPower(0);
        lb.setPower(0);
    }
}

