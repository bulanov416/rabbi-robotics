package org.gannacademy.libraries;

import android.os.Environment;

import java.io.File;
import java.text.SimpleDateFormat;

/**
 * Created by Nathan on 10/27/16.
 */
public class AndroidFileLogger {
    /**
     * WARNING: This is VERY NOT FINISHED. Do not use it in production. Use telemetry until this is finished.
     * This class is basically the same as FileLogger - but it's designed to work with android. I'll update this
     * documentation once the class is up and running - this should be our new logging program.
     * All files save to /sdcard/4466Logs for now.
     */

    private String filename;
    File file = new File(Environment.getExternalStorageDirectory().getPath(), filename);

    public AndroidFileLogger(String filename) {
        /**
         * Constructor:
         * @param filename (required) The name of the output file.
         */
        SimpleDateFormat sdf = new SimpleDateFormat("MM-dd HH:mm:ss"); // example: 01-22 00:19:25
        this.filename = filename + sdf;

        // This next section is commented because it produces errors. TODO fix this file writer
        /* try {
            Context context = new Context() {}; // TODO figure out the Context class
            FileOutputStream outputStream = context.openFileOutput(this.filename, Context.MODE_PRIVATE);
            outputStream.write("Testing 123".getBytes());
            outputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        } */
    }
}
