package org.firstinspires.ftc.teamcode.archive.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.gannacademy.libraries.ContextFileLogger;

import java.io.IOException;

/**
 * Created by Levi on 11/1/2016.
 */
@TeleOp(name = "FileDebug")
public class FileLoggerAuto extends LinearOpMode{

    public FileLoggerAuto() {}

    @Override
    public void runOpMode() throws InterruptedException {
        ContextFileLogger logger = null;
        try {
            logger = new ContextFileLogger("TestLog");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("LOGGER FAILED", "IOException thrown. Wait 10s for stop");
            telemetry.update();
            Thread.sleep(1000);
            stop();
        }

        logger.write("Hey, can you hear me?");
        logger.write("Thought so.");

        waitForStart();

        logger.write("Is there anything wrong with your life?");
        logger.write("You can fix it with GRACIOUS PROFESSIONALISM(tm).");
        logger.write("Warning: side effects may transformation into Micah.");
        Thread.sleep(1000);
        logger.write("GRACIOUS PROFESSIONALISM(tm): not even once.");
        stop();
    }

}