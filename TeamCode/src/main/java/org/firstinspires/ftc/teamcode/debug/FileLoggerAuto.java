package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.gannacademy.libraries.FileLogger;

/**
 * Created by Levi on 11/1/2016.
 */
@TeleOp(name = "FileDebug")
public class FileLoggerAuto extends LinearOpMode{

    public FileLoggerAuto() {}

    @Override
    public void runOpMode() {
        FileLogger logger = new FileLogger("TestLog");
        logger.write("Init 1");
    }

}