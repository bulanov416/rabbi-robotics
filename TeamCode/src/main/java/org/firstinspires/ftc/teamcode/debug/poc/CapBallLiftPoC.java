package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.gannacademy.libraries.HardwareRabbi;

/**
 * Created by Nathan on 11/15/2016.
 */

@TeleOp(name = "Cap Ball Lift PoC", group = "Proofs of Concecpt")
// @Disabled
public class CapBallLiftPoC extends LinearOpMode{

    HardwareRabbi robot = new HardwareRabbi();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                stop();
            }
            driveBasic();
            // Prevents motor conflict
            if (gamepad1.dpad_up && gamepad1.dpad_down) {
                robot.lift.setPower(0);
            }
            // move the ball up
            if (gamepad1.dpad_up) {
                robot.lift.setPower(1);
            }
            // move the ball down
            if (gamepad1.dpad_down) {
                robot.lift.setPower(-1);
            }
        }
    }

    public void driveBasic() {
        float l_gp1_left_stick_y = gamepad1.left_stick_y;
        float l_left_drive_power
                = (float) scale_motor_power(l_gp1_left_stick_y);

        float l_gp1_right_stick_y = gamepad1.right_stick_y;
        float l_right_drive_power
                = (float) scale_motor_power(l_gp1_right_stick_y);

        robot.r.setPower(l_right_drive_power);
        robot.l.setPower(l_left_drive_power);
        robot.rb.setPower(l_right_drive_power);
        robot.lb.setPower(l_left_drive_power);
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
}
