package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;

import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import android.util.Log;

/**
 * This class provides image processing functionalities for Near Target Identification.
 * It inherits from KiboRpcService to use the provided API for communication with the Kibo Robot Programming Challenge.
 */
public class ImageProcessing {

    /**
     * Tag used for logging purposes. It helps identify log messages from this class.
     */
    private final String TAG = this.getClass().getSimpleName();

    /**
     * Counter to keep track of how many times the imageProcessing method has been called.
     */
    private int imageProcessing_called = 0;

    // initialise objects to be used in image processing
    private HashMap<Integer, Position> arucoTargets;
    private DetectorParameters detectorParameters;
    public List<Mat> corners;
    private Dictionary dictionary;
    private Mat ids;
    
    public ImageProcessing (Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters detectorParameters) {
        this.dictionary = dictionary;
        this.corners = corners;
        this.ids = ids;
        this.detectorParameters = detectorParameters;
    }

    /**
     * Processes the provided image using ArUco marker detection and labeling.
     *
     * @param grayImage         The input image for image processing
     * @param targetID          The target ID for which the image is being processed.
     */
    public Mat imageProcessing(Mat grayImage, int targetID) {

        Log.i(TAG+"/imageProcessing", "Image has been saved in Black and White");

        Mat colorImage = new Mat();
        // Convert the grayscale image to color
        Imgproc.cvtColor(grayImage, colorImage, Imgproc.COLOR_GRAY2BGR);

        Log.i(TAG+"/imageProcessing", "TARGET " + targetID + " image processing");
        Aruco.detectMarkers(colorImage, dictionary, corners, ids, detectorParameters);
        Aruco.drawDetectedMarkers(colorImage, corners, ids, new Scalar( 0, 255, 0 ));

        Imgproc.putText(colorImage, "Aruco:"+ Arrays.toString(ids.get(0,0)), new org.opencv.core.Point(30.0, 80.0), 3, 0.5, new Scalar(255, 0, 0, 255), 1);
        Log.i(TAG+"imageProcessing", "Aruco marker has been labeled");

        imageProcessing_called++;
        return colorImage;

    }

    public double[] inspectCorners(List<Mat> corners) {
        /* Ideas:
        * 1) detect AR tag and its ID
        *   - detect centre of AR tag (AR_midpoint)
        * 2) detect its placement
        * 3) make it move by few cm (ref to discord) to get to aruco_middle
        *
        * param:
        * 1) aruco_middle: middle of the circle
        * 2) AR_midpoint: middle of the AR Tag
        * */

        double aruco_middle_x = 0.0;
        double aruco_middle_y = 0.0;
        double AR_midpoint;

        double[] topright;
        double[] topleft;
        double[] bottomleft;
        double[] bottomright;

        final int x_coords = 0;
        final int y_coords = 1;

        try{

        bottomleft  = corners.get(0).get(0, 2);
        bottomright = corners.get(0).get(0, 3);
        topleft     = corners.get(0).get(0, 1);
        topright    = corners.get(0).get(0, 0);

        aruco_middle_x = (bottomleft[x_coords] + bottomright[x_coords] + topleft[x_coords] + topright[x_coords])/4;
        aruco_middle_y = (bottomleft[y_coords] + bottomright[y_coords] + topleft[y_coords] + topright[y_coords])/4;
        
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        double[] aruco_middle = {aruco_middle_x, aruco_middle_y};

        return aruco_middle;
    }

    public Point moveCloserToArucoMarker(Kinematics kinematics, double[] aruco_middle, int current_target){
        int counter_x = 0;
        int counter_y = 0;
        final double middle_x = 1280/2;
        final double middle_y = 960/2;

        double aruco_middle_x = aruco_middle[0];
        double aruco_middle_y = aruco_middle[1];

        double x_difference = middle_x - aruco_middle_x;
        Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is: " + x_difference);
        double y_difference = middle_y - aruco_middle_y;
        Log.i(TAG+"/moveCloserToArucoMarker", "The y difference is: " + y_difference);

        Point point;
        Point new_point;

        point = kinematics.getPosition();

        // initialize new_point
        new_point = new Point (point.getX(), point.getY(), point.getZ());

        /* We want the bee to move closer, thus each 4 points might be a bit different
         * Thought process:
         * 1) check for the x difference & y difference
         * 2) if the x diff & y diff still less than 20, repeat the image processing few times
          * */

        /* Case 1: positive difference case */
        while (x_difference >= 20){
            /* special case if x >= 100 */
            if (x_difference >= 100){
                new_point = new Point (point.getX() + (x_difference - 80), point.getY(), point.getZ());
                x_difference -= 80;
            }
            else {
                new_point = new Point (point.getX() + (x_difference - 20), point.getY(), point.getZ());
                x_difference -= 20;
            }
            counter_x ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is currently: " + x_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (x) attempted: " + counter_x);
        }

        while (y_difference >= 20){
            /* special case if y >= 100 */
            if (y_difference >= 100){
                new_point = new Point (point.getX(), point.getY() + (y_difference - 80), point.getZ());
                y_difference -= 80;
            }
            else {
                new_point = new Point (point.getX(), point.getY() + (y_difference - 20), point.getZ());
                y_difference -= 20;
            }
            counter_y ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The y difference is currently: " + y_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (y) attempted: " + counter_y);
        }

        /* Case 2: negative difference case */
        while (x_difference <= -20){
            /* special case if x >= 100 */
            if (x_difference <= 100){
                new_point = new Point (point.getX() + (x_difference + 80), point.getY(), point.getZ());
                x_difference += 80;
            }
            else {
                new_point = new Point (point.getX() + (x_difference + 20), point.getY(), point.getZ());
                x_difference += 20;
            }
            counter_x ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is currently: " + x_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (x) attempted: " + counter_x);
        }

        while (y_difference <= -20){
            /* special case if y >= 100 */
            if (y_difference <= 100){
                new_point = new Point (point.getX(), point.getY() + (y_difference + 80), point.getZ());
                y_difference += 80;
            }
            else {
                new_point = new Point (point.getX(), point.getY() + (y_difference + 20), point.getZ());
                y_difference += 20;
            }
            counter_y ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The y difference is currently: " + y_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (y) attempted: " + counter_y);
        }

        return new_point;

    }


    private void init() {
        // initialise target to aruco marker hashmap
        arucoTargets = new HashMap<Integer, Position>();
        arucoTargets.put(1,Position.TopRight);
        arucoTargets.put(2,Position.TopLeft);
        arucoTargets.put(3,Position.BottomLeft);
        arucoTargets.put(4,Position.BottomRight);
        arucoTargets.put(5,Position.TopRight);
        arucoTargets.put(6,Position.TopLeft);
        arucoTargets.put(7,Position.BottomLeft);
        arucoTargets.put(8,Position.BottomRight);
        arucoTargets.put(9,Position.TopRight);
        arucoTargets.put(10,Position.TopLeft);
        arucoTargets.put(11,Position.BottomLeft);
        arucoTargets.put(12,Position.BottomRight);
        arucoTargets.put(13,Position.TopRight);
        arucoTargets.put(14,Position.TopLeft);
        arucoTargets.put(15,Position.BottomLeft);
        arucoTargets.put(16,Position.BottomRight);
    }

}