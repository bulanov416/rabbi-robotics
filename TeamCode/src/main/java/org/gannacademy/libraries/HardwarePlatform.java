package org.gannacademy.libraries;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive;

/**
 * Created by Nathan on 11/21/2016.
 *
 * This is NOT an OpMode.
 *
 * This class contains all of the hardware variables for our robot.
 *
 * This class should be used in OpModes, as well as in libraries. Remember to always use the same instance of
 * this class - in libraries, get it from the instance of the class. Use a load() method that gets the
 * HardwarePlatform from the place it is called.
 * */
public class HardwarePlatform {

    public DcMotor l, r, lb, rb;

    public DcMotor lift;
    public DcMotor cap;

    public Servo claw_servo;
    public Servo button_pusher;
    public Servo cap_deploy;

    public ColorSensor beaconColor;
    public OpticalDistanceSensor eods;
    public TouchSensor beaconTouch;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public HardwarePlatform() {

    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        hardwareMap = hwMap;
        telemetry = telemetry;
        telemetry.addLine("4466 RABBI - HardwarePlatform v2.0.0-161121"); // ask Nathan about changing this number
        telemetry.update();
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        r.setDirection(DcMotor.Direction.REVERSE);
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        rb.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("Drive Motors Ready.");
        telemetry.update();
        lift = hardwareMap.dcMotor.get("lift");
        cap = hardwareMap.dcMotor.get("cap");
        telemetry.addLine("Cap Ball Motors Ready.");
        telemetry.update();
        claw_servo = hardwareMap.servo.get("clawservo");
        button_pusher = hardwareMap.servo.get("pusher");
        cap_deploy = hardwareMap.servo.get("deploy");
        telemetry.addLine("Servos Ready.");
        telemetry.update();
        beaconColor = hardwareMap.colorSensor.get("beacon beaconColor");
        eods = hardwareMap.opticalDistanceSensor.get("eods");
        telemetry.addLine("Sensors Ready.");
        telemetry.addLine("Robot Initialized and ready for use.");
        telemetry.update();
    }
}
