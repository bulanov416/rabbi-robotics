package org.firstinspires.ftc.libraries;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

/**
 * Created by Levi on 10/23/2016.
 */
public class FileLogger {

    /**
     * This class allows for file-based logging on a per-class level.
     * It forces each OpMode to have its own log, which is good logging practice in this case.
     * To use, assign a variable to a <code>new FileLogger("filename");</code>. Do not include .txt!
     **/
    private String name;
    private SimpleDateFormat sdf = new SimpleDateFormat("HH:mm MM/dd/yyyy"); // using American date format

    public FileLogger(String name) {
        /**
         * Constructor.
         * @param name (required) The name of the output file.
         */
        this.name = name + System.currentTimeMillis();
        String path = name + ".txt";
        File file = new File(path);
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {/* ignore it */}
        }
        this.write("Name: " + this.name); // once we know the logger works, we should remove this
        this.write(System.getProperty("line.separator") + "Launched at: " + sdf.format(System.currentTimeMillis()));
        this.write(System.getProperty("line.separator") + "<----------------------------------------------->");
    }


    /**
     Writes a new line to the file, and saves it.
    **/
    public void write(String line){
        try {
            String fpath = name+".txt";
            File file = new File(fpath);
            FileWriter fw = new FileWriter(file.getAbsoluteFile());
            BufferedWriter bw = new BufferedWriter(fw);
            String time = sdf.format(System.currentTimeMillis());
            line = System.getProperty("line.separator") + "[" + time + "]: " + line;
            bw.write(line);
            bw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String getName() {return this.name;}

}
