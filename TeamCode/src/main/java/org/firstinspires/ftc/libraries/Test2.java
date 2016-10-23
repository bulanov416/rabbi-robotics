package org.firstinspires.ftc.libraries;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

/**
 * Created by Levi on 10/23/2016.
 */
public class Test2 {

    /**
     This class is intended to facilitate extracting code output from FTC runtime cycles.
     It allows for each separate class to have its own output file (and in fact requires this),
     prints out a human formatted time and a String as specified by the WriteLine(String line) method.
     Simply instantiate the FTCLogger object with the desired file name (simply the name, not the type or format)
     Then, use the WriteLine() method to "print" a line to the file. Will save after each write, so
     fatal crashes should not interfere.
     **/
    private String name;
    private SimpleDateFormat sdf = new SimpleDateFormat("HH:mm dd/MMM/yyyy");

    public Test2(String name) {
        this.name = name+System.currentTimeMillis();
        String path = name+".txt";
        File file = new File(path);
        if (!file.exists()) {
            try {
                file.createNewFile();
            }
            catch (IOException e) {
            }
        }
        this.write("Log file name: " + this.name);
        this.write(System.getProperty("line.separator") + "Launched at: " + sdf.format(System.currentTimeMillis()));
        this.write(System.getProperty("line.separator") + "<----------------------------------------------->");

    }
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

    public String getName() {
        return this.name;
    }

}
