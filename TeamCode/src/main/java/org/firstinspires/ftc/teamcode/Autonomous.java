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
        import com.qualcomm.robotcore.util.Range;



/**
 * Created by alexbulanov on 12/19/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "REALSLIMAUTO")
public class Autonomous extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    OpticalDistanceSensor eods;
    ColorSensor color_left;
    ColorSensor color_right;
    Servo button_left;
    Servo button_right;
    double runtime;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        button_left = hardwareMap.servo.get("bl");
        button_left = hardwareMap.servo.get("br");
        eods = hardwareMap.opticalDistanceSensor.get("eods");
        color_left = hardwareMap.colorSensor.get("cl");
        color_right = hardwareMap.colorSensor.get("cr");
        l.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        runtime = System.currentTimeMillis();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Initiates IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        telemetry.addData("EODS LEVEL: ", eods.getLightDetected());
        telemetry.update();


        Orientation orientationI = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC)
                .toAxesOrder(AxesOrder.ZXY);
        double bearingYawD = orientationI.firstAngle; //Initial bearing on the yaw axis
        telemetry.addLine("Bearing" + Double.toString(bearingYawD));
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("EODS LEVEL: ", eods.getLightDetected());
            telemetry.update();

           /* while (opModeIsActive()) {
                drive(0.2);
                sleepOpMode(500);
                stopDrive();
                turnLeft(0.3);
                sleepOpMode(600);
                stopDrive();
                drive(0.4);
                sleepOpMode(1550);
                stopDrive();
                turnRight(0.2);
                sleepOpMode(650);
                stopDrive();
                while (eods.getLightDetected() < 0.03) {
                    drive(0.2);
                }
                stopDrive();
                drive(0.2);
                sleepOpMode(500);
                stopDrive();
                while (eods.getLightDetected() < 0.03) {
                    turnLeft(0.2);
                }
                break;
            }
        }
        */
        }
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

    public void sleepOpMode(double millTime) throws InterruptedException{
        double time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() != time + millTime) {
            Thread.sleep(1);
        }
    }

}


