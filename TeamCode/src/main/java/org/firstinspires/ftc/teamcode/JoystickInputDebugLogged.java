package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.gannacademy.libraries.FileLogger;

/**
 * Created by alexbulanov on 9/28/16.
 */
@TeleOp(name = "Joystick Input Debug Logged")
public class JoystickInputDebugLogged extends LinearOpMode {

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

    public JoystickInputDebugLogged() {

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
        FileLogger logger = new FileLogger("DrivePowerValuesLog");
        logger.write("OpMode Started");
        while (true) {
            float gamepad_left_stick_input = Range.clip(gamepad1.left_stick_y, -1, 1);
            float l_left_drive_power
                    = (float) scale_motor_power(gamepad_left_stick_input);
            float gamepad_right_stick_input = Range.clip(gamepad1.right_stick_y, -1, 1);
            float l_right_drive_power
                    = (float) scale_motor_power(gamepad_right_stick_input);

            while (gamepad_left_stick_input != 0 || gamepad_right_stick_input != 0) {
                String stick_input_text = Float.toString(gamepad_left_stick_input) + Float.toString(gamepad_right_stick_input);
                String scaled_power_text = Float.toString(l_left_drive_power) + Float.toString(l_right_drive_power);
                logger.write(stick_input_text);
                logger.write(scaled_power_text);
            }
        }
    }
}