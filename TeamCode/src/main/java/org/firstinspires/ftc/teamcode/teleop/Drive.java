package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.libraries.FileLogger;

/**
 * Created by alexbulanov on 9/28/16.
 */
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
   /* DcMotor fly1;
    DcMotor fly2;
    Servo bring;
    Servo clasp1;
    Servo clasp2;
*/

    public Drive() {

    }

    double scale_motor_power(double p_power)  //Scales joystick value to output appropriate motor power
    {                                          //Use like "scale_motor_power(gamepad1.left_stick_x)"
        //
        // Assume no scaling.
        //
        double l_scale = 0.0;

        //
        // Ensure the values are legal.
        //
        double l_power = Range.clip(p_power, -1, 1);

        double[] l_array =
                {0.00, 0.05, 0.09, 0.10, 0.12
                        , 0.15, 0.18, 0.24, 0.30, 0.36
                        , 0.43, 0.50, 0.60, 0.72, 0.85
                        , 1.00, 1.00
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);
        if (l_index < 0) {
            l_index = -l_index;
        } else if (l_index > 16) {
            l_index = 16;
        }

        if (l_power < 0) {
            l_scale = -l_array[l_index];
        } else {
            l_scale = l_array[l_index];
        }

        return l_scale;

    }

    @Override
    public void runOpMode() {
        FileLogger logger = new FileLogger("OpModeDriveLog");
        logger.write("test");
        while (true) {

            logger.write("While Loop Cycle");
            l = hardwareMap.dcMotor.get("l");
            r = hardwareMap.dcMotor.get("r");
            rb = hardwareMap.dcMotor.get("rb");
            lb = hardwareMap.dcMotor.get("lb");
       /* fly1 = hardwareMap.dcMotor.get("fly1");
        fly2 = hardwareMap.dcMotor.get("fly2");
        bring = hardwareMap.servo.get("bring");
        clasp2 = hardwareMap.servo.get("clasp2");
        clasp1 = hardwareMap.servo.get("clasp1");
*/
            //fly2.setDirection(DcMotor.Direction.REVERSE);
            r.setDirection(DcMotor.Direction.REVERSE);
            rb.setDirection(DcMotor.Direction.REVERSE);

            float l_gp1_left_stick_y = gamepad1.left_stick_y;
            float l_left_drive_power
                    = (float) scale_motor_power(l_gp1_left_stick_y);

            float l_gp1_right_stick_y = gamepad1.right_stick_y;
            float l_right_drive_power
                    = (float) scale_motor_power(l_gp1_right_stick_y);

            r.setPower(l_right_drive_power);
            l.setPower(l_left_drive_power);
            rb.setPower(l_right_drive_power);
            lb.setPower(l_left_drive_power);
          /*  while (gamepad1.dpad_up) {
                fly1.setPower(1);
                fly2.setPower(1);
            }


            //When button a is pressed, the ball function is executed.
            if (gamepad2.dpad_down) {

                bring.setPosition(1);

            }

            if (gamepad2.dpad_up) {

                clasp2.setPosition(0);

            }

            if (gamepad2.dpad_left) {

                clasp2.setPosition(1);
                clasp1.setPosition(1);

            }

            if (gamepad2.dpad_right) {

                clasp2.setPosition(0);
                clasp1.setPosition(0);

            }
            if (!gamepad1.dpad_down) {
                fly1.setPower(0);
                fly2.setPower(0);
            }
            fly1.setPower(0);
            fly2.setPower(0);
        }
        /*clasp2.setPosition(1);
        sleep(1000);
        bring.setPosition(0.9);
        sleep(2000);
        clasp1.setPosition(0.5);
        clasp2.setPosition(0.5);*/
        }

        //This is the function to pick up balls and bring them to the launcher
       /* public void ball() {

        clasp1.setPosition(1);
        clasp2.setPosition(1);
        sleep(1000);
        bring.setPosition(0.9);
        sleep(2000);
        clasp1.setPosition(0.5);
        clasp2.setPosition(0.5);

    }
*/
        //This function should be used for sleep because it is setup to throw InterruptedException.

    }


}