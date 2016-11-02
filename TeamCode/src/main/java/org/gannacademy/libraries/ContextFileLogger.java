package org.gannacademy.libraries;

import android.app.Application;
import android.content.Context;
import android.content.res.Configuration;
import android.os.Environment;
import android.view.ViewParent;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

/**
 * Created by Nathan on 11/1/16.
 */
/*
This class is not named correctly. We should rename it.
For now, deal with it.
 */
public class ContextFileLogger extends Application {

    SimpleDateFormat sdf = new SimpleDateFormat("MM-dd-yyyy--HH:mm:ss");
    File file;

    public ContextFileLogger(String filename) throws IOException {
        File dirStorage = null;
        if (isExternalStorageWritable()) {
            String path = Environment.getExternalStorageDirectory().toString() + "/4466Logs";
            dirStorage = new File(path);
            if (!dirStorage.isDirectory()) {
                dirStorage.mkdir();
            }
        } else {
            throw new IOException("External storage is not writable.");
        }
        // format the filename
        SimpleDateFormat sdf = new SimpleDateFormat("MM-dd-yyyy--HH:mm:ss");
        filename = sdf + " " + filename;
        // create the file
        File file = new File(dirStorage, filename);
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void write(String line){
        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter(file.getAbsoluteFile()));
            String time = sdf.format(System.currentTimeMillis());
            line = System.getProperty("line.separator") + "[" + time + "]: " + line;
            bw.write(line);
            bw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String generateErrorReport() {
        // TODO make a thing that runs through and checks exactly what went wrong
        String errorDescription = "Whoops."; // we'll modify this variable
        return errorDescription;
    };

    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

}
