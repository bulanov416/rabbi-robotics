package org.gannacademy.libraries;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
 * Sensor: Optical Distance Sensor           "ods"
 * Sensor: Ultrasonic Distance Sensor       "uds"
 *
 * More motors can be added as they are implemented
 */
public class HardwareRabbi {
    // Motors

    public DcMotorController frontMotorController; // controller vars are good to have, might be useful
    public DcMotor l; // motor vars are grouped by their controller
    public DcMotor r;
    public DcMotorController backMotorController;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotorController capBallLiftController;
    public DcMotor capBallLift;
    // Servos
    public ServoController   buttonPusherController;
    public Servo   buttonPushServo;

    // Sensors
    public DeviceInterfaceModule coreDeviceInterface;
    public ColorSensor           color;
    public OpticalDistanceSensor eods;
    public TouchSensor           buttonPusherTouch;
    // Legacy Sensors
    public LegacyModule     legacyModule;
    public LightSensor      light;
    public UltrasonicSensor uds;

    // This class uses the HardwareMap and Telemetry from the OpMode that it is instantiated in.
    private HardwareMap hwMap;
    private Telemetry telemetry;

    private ElapsedTime period  = new ElapsedTime(); // for internal use

    /* Enum ButtonPushPositions {
        LEFT_BUTTON, REST_POSITION, RIGHT_BUTTON;
    }; */

    /* Constructor */
    public HardwareRabbi(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry opModeTelemetry) {
        hwMap = ahwMap; // uses the HardwareMap from the OpMode
        telemetry = opModeTelemetry; // allows HardwareRabbi to use telemetry in the context of the opmode

        // frontMotorController = hwMap.dcMotorController.get("FrontMotorController");
        // backMotorController = hwMap.dcMotorController.get("BackMotorController");
        // capBallLiftController = hwMap.dcMotorController.get("CapBallLiftController");

        // Initialize Motors
        l  = hwMap.dcMotor.get("l");
        lb = hwMap.dcMotor.get("lb");
        r  = hwMap.dcMotor.get("r");
        rb = hwMap.dcMotor.get("rb");
        capBallLift = hwMap.dcMotor.get("capBallLift");
        // Set motor direcrion
        l .setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        r .setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        capBallLift.setDirection(DcMotor.Direction.FORWARD);
        // Initially stop all motors
        stopDriving();
        // Make sure the motors are not expecting encoders
        l .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capBallLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the servo
        buttonPusherController = hwMap.servoController.get("ButtonPusherController");
        buttonPushServo = hwMap.servo.get("buttonPushServo");
        // Set it's position
        buttonPushServo.setPosition(90); // replace with vars
        // Initialize the sensors
        color = hwMap.colorSensor.get("color");
        eods = hwMap.opticalDistanceSensor.get("eods");

        // Initialize the legacy sensors
        //light = hwMap.lightSensor.get("ls");
        //light.enableLed(false); // why is this here?

        //uds = hwMap.ultrasonicSensor.get("uds");
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
    public void driveCentimeters (double distance, double power) throws InterruptedException {
        // drives a distance in cm based on a conversion factor from seconds
        Range.clip(power, -1, 1);
        double conversionFactor = 30; // cm/s
        double distanceInSeconds = distance / conversionFactor;
        telemetry.addData(Double.toString(distanceInSeconds), Double.toString(power));
        telemetry.update();
        driveSeconds(power, distanceInSeconds / power);
        // testing a way of driving at lower powers
    }

    /***
     * Drives the robot for a certain amount of time, at a certain power. Time is in Seconds, power is
     * between 1 and -1. Negative power drives the robot backwards.
     *
     * @param power  the power to run the motors at (between -1 and 1)
     * @param time  the amount of seconds to run the motors for (positive number)
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
        Thread.sleep((long) time * 1000);
        stopDriving();
    }

    /***
     * Starts driving at a certain power. stopDriving must be called manually afterwards.
     *
     * @param power the power to run the motor at (between -1 and 1)
     */
    public void drive(double power) {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(power);
        lb.setPower(power);
    }

    /**
     * Turns left at a specified power, for a specified time.
     * @param power Desired power of turn
     * @param time Desired time of turning
     * @throws InterruptedException
     */
    public void turnLeft(double power, double time) throws InterruptedException {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(-power);
        lb.setPower(-power);
        Thread.sleep((long) (time * 1000));
        stopDriving();
    }

    /**
     * Turns right at a specified power, for a specified time.
     * @param power Desired power of turn
     * @param time Desired time of turning
     * @throws InterruptedException
     */
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

    /**
     * Depracated. Use turnLeft() instead.
     * @param power Motor power
     */
    public void setLeftPower(double power) {
        r.setPower(power);
        rb.setPower(power);
        l.setPower(-power);
        lb.setPower(-power);
    }
    /**
     * Deprecated. Use turnRight() instead.
     * @param power Motor power
     */
    public void setRightPower(double power) {
        r.setPower(-power);
        rb.setPower(-power);
        l.setPower(power);
        lb.setPower(power);
    }

    /* ENCODERS */
    private void driveMotorWithEncoders(DcMotor motor, double power, int ticks) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        while (motor.isBusy()) {}
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnRightTicks(double power, int ticks) {
        driveMotorWithEncoders(r, power, ticks);
        rb.setPower(power);
        driveMotorWithEncoders(l, -power, ticks);
        lb.setPower(-power);
        while (r.isBusy()) {}
        stopDriving();
    }
    public void turnLeftTicks(double power, int ticks) {
        driveMotorWithEncoders(l, power, ticks);
        lb.setPower(power);
        driveMotorWithEncoders(r, -power, ticks);
        rb.setPower(-power);
        while (r.isBusy()) {}
        stopDriving();
    }
    /**
     * Drives at a specified power, for a specified amount of ticks, using encoders.
     * @param power Desired power of turn
     * @param ticks Desired number of ticks
     */
    public void driveTicks(double power, int ticks) {
        driveMotorWithEncoders(r, power, ticks);
        rb.setPower(power);
        driveMotorWithEncoders(l, -power, ticks);
        lb.setPower(power);
        while (r.isBusy()) {}
        stopDriving();
    }

}

