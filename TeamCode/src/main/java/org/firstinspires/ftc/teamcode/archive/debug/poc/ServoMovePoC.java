package org.firstinspires.ftc.teamcode.archive.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nathan on 10/16/2016.
 */
@Autonomous(name="Servo Move Proof of Concept")
// @Disabled
public class ServoMovePoC extends LinearOpMode{

    Servo buttonpusher;
    final int[] servo_positions = {0,45,90,135};

    public ServoMovePoC() {}

    public void runOpMode() throws InterruptedException {
        buttonpusher = hardwareMap.servo.get("button pusher servo"); // connect the variable to the servo
        buttonpusher.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        // OpMode starts here
        telemetry.addData("Initial Servo Pos:", buttonpusher.getPosition());
        telemetry.update();
        buttonpusher.setPosition(scale_servo_pos(servo_positions[1]));
        telemetry.addData("Servo position 1", "LOCK");
        telemetry.update();
        Thread.sleep(200);
        buttonpusher.setPosition(scale_servo_pos(servo_positions[2]));
        telemetry.addData("Servo position 2", "LOCK");
        telemetry.update();
        Thread.sleep(200);
        buttonpusher.setPosition(scale_servo_pos(servo_positions[3]));
        telemetry.addData("Servo position 3", "LOCK");
        telemetry.update();
        Thread.sleep(200);
        buttonpusher.setPosition(scale_servo_pos(servo_positions[0]));
        telemetry.addData("Servo Test", "COMPLETE");
    }

    public int scale_servo_pos(int deg_pos) {
        // A method for scaling the angular rotation of the servo to its range
        return (int) Range.scale(deg_pos, 0, 165, 0, 255);
    }

}
