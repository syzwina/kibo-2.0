package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.core.Scalar;
import org.opencv.aruco.DetectorParameters;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import android.util.Log;

import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

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
    private HashMap<Integer, Integer> arucoTargets;
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

        // once you choose one ID
        // decide which ID it is, and were it is relative to the centre of the circle
        // and set the new 'centre' coordinate to 'aruco_middle'

        // use mod 4 to get whether it is tl, tr, bl, br


        double[] topright;
        double[] topleft;
        double[] bottomleft;
        double[] bottomright;

        double aruco_middle_x = 0.0;
        double aruco_middle_y = 0.0;

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
            new_point = new Point (point.getX() + (x_difference - 20), point.getY(), point.getZ());

            /* add the x_difference so the while loop can be break */
            x_difference -= 10;
            counter_x ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (x) attempted: " + counter_x);
            Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is currently: " + x_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
        }

        while (y_difference >= 20){
            new_point = new Point (point.getX(), point.getY() + (y_difference - 20), point.getZ());

            /* add the y_difference so the while loop can be break */
            y_difference -= 10;
            counter_y ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (y) attempted: " + counter_y);
            Log.i(TAG+"/moveCloserToArucoMarker", "The y difference is currently: " + y_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
        }

        /* Case 2: negative difference case */
        while (x_difference <= -20){
            new_point = new Point (point.getX() + (x_difference + 20), point.getY(), point.getZ());

            /* add the x_difference so the while loop can be break */
            x_difference += 10;
            counter_x ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (x) attempted: " + counter_x);
            Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is currently: " + x_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
        }

        while (y_difference <= -20){
            new_point = new Point (point.getX(), point.getY() + (y_difference + 20), point.getZ());

            /* add the y_difference so the while loop can be break */
            y_difference += 10;
            counter_y ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (y) attempted: " + counter_y);
            Log.i(TAG+"/moveCloserToArucoMarker", "The y difference is currently: " + y_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
        }

        return new_point;

    }


    private void init() {

        // initialise aruco dictionary
        dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        // initialise objects to be used in image processing
        corners = new ArrayList<Mat>();
        ids = new Mat(1, 4, 1, new Scalar( 0, 150, 250 ));
        detectorParameters = DetectorParameters.create();

        // initialise target to aruco marker hashmap
        arucoTargets = new HashMap<Integer, Integer>();
        arucoTargets.put(1,1);
        arucoTargets.put(2,1);
        arucoTargets.put(3,1);
        arucoTargets.put(4,1);
        arucoTargets.put(5,2);
        arucoTargets.put(6,2);
        arucoTargets.put(7,2);
        arucoTargets.put(8,2);
        arucoTargets.put(9,3);
        arucoTargets.put(10,3);
        arucoTargets.put(11,3);
        arucoTargets.put(12,3);
        arucoTargets.put(13,4);
        arucoTargets.put(14,4);
        arucoTargets.put(15,4);
        arucoTargets.put(16,4);

    }

}