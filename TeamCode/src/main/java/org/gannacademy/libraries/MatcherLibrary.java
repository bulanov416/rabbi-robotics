package org.gannacademy.libraries;

import android.app.Activity;
import android.content.Intent;
import android.provider.MediaStore;

import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Levi on 11/3/2016.
 */
public class MatcherLibrary {

    /**
     * @param inFile Input file, WIP
     * @param templateFile Match file, WIP
     * @param outFile Out file, WIP
     * @param match_method Match method, use 5 or 3
     * @param fov Field of Vision of image
     * Returns positive angles if match is situated counter-clockwise, negative if clockwise
     **/

    public double run(String inFile, String templateFile, String outFile, int match_method, int fov)  {

        System.loadLibrary("resources");

        Mat img = Highgui.imread(inFile); //Will be replaced with direct camera capture method
        Mat templ = Highgui.imread(templateFile);

        // / Creates the result matrix
        int result_cols = img.cols() - templ.cols() + 1;
        int result_rows = img.rows() - templ.rows() + 1;
        Mat result = new Mat(result_rows, result_cols, CvType.CV_32FC1);

        // / Matching and normalization
        Imgproc.matchTemplate(img, templ, result, match_method);
        Core.normalize(result, result, 0, 1, Core.NORM_MINMAX, -1, new Mat());

        // / Localizing the best match with minMaxLoc
        MinMaxLocResult mmr = Core.minMaxLoc(result);

        Point matchLoc;
        if (match_method == Imgproc.TM_SQDIFF || match_method == Imgproc.TM_SQDIFF_NORMED) {
            matchLoc = mmr.minLoc;
        } else {
            matchLoc = mmr.maxLoc;
        }

        //Places box around template match for debug purposes
        Core.rectangle(img, matchLoc, new Point(matchLoc.x + templ.cols(),
                matchLoc.y + templ.rows()), new Scalar(0, 255, 0));

        //Center of Image, aligned with center of phone, and therefor bot.
        double xMid = img.cols()/2;

        //Poor calculation of angle, will be sufficient (hopefully)
        double angleOffCenter = (xMid/(matchLoc.x - xMid))*(fov/2);

        // Save the visualized detection.
        Highgui.imwrite(outFile, img);

        //Returns Angle
        return angleOffCenter;
    }

}