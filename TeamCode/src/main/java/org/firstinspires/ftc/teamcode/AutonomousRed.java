package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.hardware.adafruit.BNO055IMU;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.util.Range;



/**
 * Created by alexbulanov on 12/19/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "REALSLIMAUTO")
public class AutonomousRed extends LinearOpMode {


    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    OpticalDistanceSensor eods;
    ColorSensor color_left;
    ColorSensor color_right;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    //double runtime;
    //BNO055IMU imu;
    TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        eods = hardwareMap.opticalDistanceSensor.get("eods");
        color_left = hardwareMap.colorSensor.get("cl");
        color_right = hardwareMap.colorSensor.get("cr");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        //runtime = System.currentTimeMillis();
        touch = hardwareMap.touchSensor.get("t");
        wall_servo = hardwareMap.servo.get("ws");
       /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Initiates IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/
        telemetry.addData("EODS LEVEL: ", eods.getLightDetected());
        telemetry.update();

        waitForStart();

        wall_servo.setPosition(0.37);
        button_right.setPosition(0.1);
        button_left.setPosition(0.9);
        color_left.enableLed(false);
        color_right.enableLed(false);
        while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
            drive(0.2);
        }
        stopDrive();
        drive(0.2);
        sleep(100);
        stopDrive();
        while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
            setLeftPower(0.18);
            setRightPower(-0.18);
        }

        while (eods.getLightDetected() > 0.03 && opModeIsActive()) {
        }

        stopDrive();
        while (!touch.isPressed()) {
            while (eods.getLightDetected() < 0.03 && !touch.isPressed() && opModeIsActive()) {
                setRightPower(0.14);
            }
            stopDrive();
            while (eods.getLightDetected() > 0.03 && !touch.isPressed() && opModeIsActive()) {
                setLeftPower(0.12);
            }
            stopDrive();
        }
        stopDrive();
        boolean colorLeftSide = isSensorRed("left");
        //insert beacon pushing code here
        drive(-0.12);

        sleep(600);
        stopDrive();
        wall_servo.setPosition(0.1);
        sleep(550);
        if (colorLeftSide) {
            button_left.setPosition(0.05);
        }
        else {
            button_right.setPosition(0.95);
        }
        while (eods.getLightDetected() > 0.03 && opModeIsActive()) {
            setRightPower(0.12);
        }
        stopDrive();
        drive(0.15);
        sleep(600);
        stopDrive();
        //Second Beacon
        drive(-0.2);
        sleep(400);
        stopDrive();
        wall_servo.setPosition(0.37);
        button_right.setPosition(0.1);
        button_left.setPosition(0.9);
        setLeftPower(-0.18);
        setRightPower(0.18);
        sleep(1850);
        while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
            drive(0.15);
        }
        stopDrive();
        drive(0.2);
        sleep(100);
        stopDrive();
        while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
            setLeftPower(0.18);
            setRightPower(-0.18);
        }

        while (eods.getLightDetected() > 0.03 && opModeIsActive()) {
        }

        stopDrive();
        while (!touch.isPressed()) {
            while (eods.getLightDetected() < 0.03 && !touch.isPressed() && opModeIsActive()) {
                setRightPower(0.14);
            }
            stopDrive();
            while (eods.getLightDetected() > 0.03 && !touch.isPressed() && opModeIsActive()) {
                setLeftPower(0.12);
            }
            stopDrive();
        }
        stopDrive();
        sleep(300);
        boolean colorRightSide = isSensorRed("left");
        //insert beacon pushing code here
        drive(-0.12);
        sleep(600);
        stopDrive();
        wall_servo.setPosition(0.1);
        sleep(550);
        if (colorRightSide) {
            button_left.setPosition(0.05);
        }
        else {

            button_right.setPosition(0.95);
        }
        while (eods.getLightDetected() > 0.03 && opModeIsActive()) {
            setRightPower(0.12);
        }
        stopDrive();
        drive(0.15);
        sleep(600);
        stopDrive();
    }


    public void drive(double power) {
        l.setPower(power);
        r.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
    }

    public void stopDrive() {
        drive(0);
    }

    public void turnLeft(double power) {
        l.setPower(-power);
        lb.setPower(-power);
        r.setPower(power);
        rb.setPower(power);
    }

    public void turnRight(double power) {
        l.setPower(power);
        lb.setPower(power);
        r.setPower(-power);
        rb.setPower(-power);
    }

    public void setLeftPower(double power) {
        l.setPower(power);
        lb.setPower(power);
    }

    public void setRightPower(double power) {
        r.setPower(power);
        rb.setPower(power);
    }

    /*
    public void sleepOpMode(double millTime) throws InterruptedException {
        double time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() != time + millTime) {
            Thread.sleep(1);
        }
    }
    */

    public boolean isSensorRed(String side) {
        String left = "left";
        ColorSensor sensor = side.equalsIgnoreCase(left) ? color_left : color_right;
        /* String colorValues = Integer.toString(sensor.argb());
        int red, blue;
        if (colorValues == "") { // this occurs when the color changes too quickly
            red = 0;
            blue = 0;
        } else {
            // extract the red and blue values from the string
            red = Integer.valueOf(colorValues.substring(2, 4));
            blue = Integer.valueOf(colorValues.substring(6, 8));
        }

        if (red == 0 || blue == 0) return false; */
        return sensor.red() > sensor.blue() ? true : false;
    }
}


