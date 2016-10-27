package org.firstinspires.ftc.teamcode.debug;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.libraries.FileLogger;

/**
 * Created by Levi on 10/25/2016.
 * This is for debug only, create a new class in this package starting with "Dev",
 * and insert whatever code snippets desired. Intended to avoid the
 * main folder getting overly disorganized. Remember to register
 * in FtcOpModeRegister.
 */
@TeleOp(name = "DevExample")
public class DevLog extends LinearOpMode {

    public DevLog() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        FileLogger logger = new FileLogger("DevExampleLog");
        logger.write("This is an example log file!");
    }
}
