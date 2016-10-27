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
     * This class allows for file-based logging on a per-class level, instantiate once only, or will override.
     * Each OpMode must have its own instance and logfile, please include the OpMode to which a log file belongs
     * in its name. To use, instantiate object FileLogger with desired "name". Do not include .txt!
     **/
    private String name;
    private SimpleDateFormat sdf = new SimpleDateFormat("HH:mm MM/dd/yyyy");

    public FileLogger(String name) {
        /**
         * Constructor.
         * @param name (required) The name of the output file, without formatting info
         */
        this.name = name + System.currentTimeMillis();
        String path = name + ".txt";
        File file = new File(path);
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println("Hopefully this prints");
            }
        }
        this.write("Name: " + this.name); // This should be left, helps to make the log file appear organized and legible.
        this.write(System.getProperty("line.separator") + "Launched at: " + sdf.format(System.currentTimeMillis()));
        this.write(System.getProperty("line.separator") + "<----------------------------------------------->");
    }


    /**
     Writes a new line to the file, and saves it, with time attached in human legible form.
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
