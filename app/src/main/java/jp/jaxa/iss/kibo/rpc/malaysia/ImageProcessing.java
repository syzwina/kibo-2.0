package jp.jaxa.iss.kibo.rpc.malaysia;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;

import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;

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
    private HashMap<Integer, Integer> arucoTargets;
    private HashMap<Integer, Position> arucoIds;
    private DetectorParameters detectorParameters;
    public List<Mat> corners;
    private Dictionary dictionary;
    private Mat ids;
    
    public ImageProcessing (Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters detectorParameters) {
        this.dictionary = dictionary;
        this.corners = corners;
        this.ids = ids;
        this.detectorParameters = detectorParameters;
        init();
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

    public double[] inspectCorners(List<Mat> corners, int current_target) {
        /* Ideas:
        * 1) detect AR tag and its ID
        * 2) detect its placement
        * 3) make it move by few cm (ref to discord) to get to aruco_middle
        *
        * param:
        * 1) aruco_middle_(x/y): middle of the circle
        * */

        double aruco_middle_x = 0;
        double aruco_middle_y = 0;

       // Aruco.estimatePoseSingleMarkers(List<Mat> corners, 5, cameraMatrix, distCoeffs, rvecs, tvecs );


       // if the aruco id, belongs to the current target, go into if block, else continue
       for ( int i=0; i<ids.rows(); i++ ) {
        if (current_target == arucoTargets.get((ids.get(i, 0)[0]))) {

            // if aruco id is at Position.A, go into A block 
            /* it needs to only use either one of the AR tags
            TR TL BL BR
            detect the positions of AR tags */
            if(arucoIds.get((ids.get(i, 0)[0])) == Position.TopRight){
                aruco_middle_x = -10.0;
                aruco_middle_y = -3.75;
                Log.i(TAG+"/inspectCorners", "It uses the Top Right Tag");
            }
            else if(arucoIds.get((ids.get(i, 0)[0])) == Position.TopLeft){
                aruco_middle_x = +10.0;
                aruco_middle_y = -3.75;
                Log.i(TAG+"/inspectCorners", "It uses the Top Left Tag");
            }
            else if(arucoIds.get((ids.get(i, 0)[0])) == Position.BottomLeft){
                aruco_middle_x = +10.0;
                aruco_middle_y = +3.75;
                Log.i(TAG+"/inspectCorners", "It uses the Bottom Left Tag");
            }
            else if (arucoIds.get((ids.get(i, 0)[0])) == Position.BottomRight) {
                aruco_middle_x = -10.0;
                aruco_middle_y = -3.75;
                Log.i(TAG+"/inspectCorners", "It uses the Bottom Right Tag");
            }
            else {
                Log.e(TAG+"/inspectCorners", "Error: unable to use any corners");
            }


        }
        else {
            continue;
        }
       }

        // TODO: check the id of aruco markers
        // TODO: compute aruco markers of the current target only
        // int id = (int)(ids.get(i, 0)[0]); // Mat object

        /*
        Previous:
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
        */

        //eventual solution will be the aruco middle
        double[] aruco_middle = {aruco_middle_x, aruco_middle_y};
        return aruco_middle;
    }

    public Point moveCloserToArucoMarker(Kinematics kinematics, int current_target){

        double[] aruco_middle = inspectCorners(corners, current_target);

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

        // shows you the position of the id
        arucoIds = new HashMap<Integer, Position>();
        arucoIds.put(1,Position.TopRight);
        arucoIds.put(2,Position.TopLeft);
        arucoIds.put(3,Position.BottomLeft);
        arucoIds.put(4,Position.BottomRight);
        arucoIds.put(5,Position.TopRight);
        arucoIds.put(6,Position.TopLeft);
        arucoIds.put(7,Position.BottomLeft);
        arucoIds.put(8,Position.BottomRight);
        arucoIds.put(9,Position.TopRight);
        arucoIds.put(10,Position.TopLeft);
        arucoIds.put(11,Position.BottomLeft);
        arucoIds.put(12,Position.BottomRight);
        arucoIds.put(13,Position.TopRight);
        arucoIds.put(14,Position.TopLeft);
        arucoIds.put(15,Position.BottomLeft);
        arucoIds.put(16,Position.BottomRight);

        // shows you which target the id belongs to
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