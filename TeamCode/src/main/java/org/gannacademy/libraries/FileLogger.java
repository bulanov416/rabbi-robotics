package org.gannacademy.libraries;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.security.Permission;
import java.text.SimpleDateFormat;
import java.util.jar.Manifest;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Environment;

/**
 * Created by Levi on 10/23/2016.
 */
public class FileLogger extends Activity {

    /**
     * This class allows for file-based logging on a per-class level, instantiate once only, or will override.
     * Each OpMode must have its own instance and logfile, please include the OpMode to which a log file belongs
     * in its name. To use, instantiate object FileLogger with desired "name". Do not include .txt!
     **/
    private String name;
    private SimpleDateFormat sdf = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss");
    private SimpleDateFormat sdf2 = new SimpleDateFormat("MM-dd-yyyy--HH:mm:ss");
    private String path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS).toString()+"/4466Logs";

    @TargetApi(Build.VERSION_CODES.M)
    public FileLogger(String name) {
        /**
         * Constructor.
         * @param name (required) The name of the output file, without formatting info
         */

        File dirStorage = new File(path);

        int storagePermission = checkSelfPermission(android.Manifest.permission.WRITE_EXTERNAL_STORAGE);

        if (storagePermission == PackageManager.PERMISSION_GRANTED) {
            if (!dirStorage.isDirectory()) {
                dirStorage.mkdir();
            }

            this.name = name + sdf2.format(System.currentTimeMillis()) + ".txt";

            File file = new File(dirStorage, name);
            if (!file.exists()) {
                try {
                    file.createNewFile();
                } catch (IOException e) { e.printStackTrace(); }
            }
            else {
                file.delete();
                try {
                    file.createNewFile(); } catch (IOException e) { e.printStackTrace();
                }
            }

            this.write("Name: " + this.name); // This should be left, helps to make the log file appear organized and legible.
            this.write(System.getProperty("line.separator") + "Launched at: " + sdf.format(System.currentTimeMillis()));
            this.write(System.getProperty("line.separator") + "<----------------------------------------------->");
        }
        else {
            System.out.println("Error - Insufficient Permissions to edit programmed directory.");
            throw new RuntimeException();
        }
    }


    /**
     Writes a new line to the file, and saves it, with time attached in human legible form.
    **/
    public void write(String line){
        try {
            File file = new File(path, name);
            BufferedWriter bw = new BufferedWriter(new FileWriter(file.getAbsoluteFile()));
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
