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
        Log.i(TAG+"/imageProcessing", "Aruco marker has been labeled");

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
        double aruco_middle_z = 0;

        // Aruco.estimatePoseSingleMarkers(List<Mat> corners, 5, cameraMatrix, distCoeffs, rvecs, tvecs );


        // if the aruco id, belongs to the current target, go into if block, else continue
        for ( int i=0; i<ids.rows(); i++ ) {

            Log.i(TAG+"/inspectCorners", "ids.rows(): " + ids.rows());
            Log.i(TAG+"/inspectCorners", "ids.size(): " + ids.size());
            Log.i(TAG+"/inspectCorners", "i: " + i);

            if (current_target == arucoTargets.get(((int) ids.get(i, 0)[0]))) {

                // if aruco id is at Position.A, go into A block 
                /* it needs to only use either one of the AR tags
                TR TL BL BR
                detect the positions of AR tags */
                if(arucoIds.get(((int) ids.get(i, 0)[0])) == Position.TopRight){
                    double[] topright = corners.get(i).get(0, 0);
                    aruco_middle_x = topright[0] - 10.0;
                    aruco_middle_z = topright[1] - 3.75;
                    Log.i(TAG+"/inspectCorners", "It uses the Top Right Tag");
                }
                else if(arucoIds.get(((int) ids.get(i, 0)[0])) == Position.TopLeft){
                    double[] topleft = corners.get(i).get(0, 1);
                    aruco_middle_x = topleft[0] + 10.0;
                    aruco_middle_z = topleft[1] - 3.75;
                    Log.i(TAG+"/inspectCorners", "It uses the Top Left Tag");
                }
                else if(arucoIds.get(((int) ids.get(i, 0)[0])) == Position.BottomLeft){
                    double[] bottomleft = corners.get(i).get(0, 2);
                    aruco_middle_x = bottomleft[0] + 10.0;
                    aruco_middle_z = bottomleft[1] + 3.75;
                    Log.i(TAG+"/inspectCorners", "It uses the Bottom Left Tag");
                }
                else if (arucoIds.get(((int) ids.get(i, 0)[0])) == Position.BottomRight) {
                    double[] bottomright = corners.get(i).get(0, 3);
                    aruco_middle_x = bottomright[0] - 10.0;
                    aruco_middle_z = bottomright[1] - 3.75;
                    Log.i(TAG+"/inspectCorners", "It uses the Bottom Right Tag");
                }
                else {
                    Log.e(TAG+"/inspectCorners", "Error: unable to use any corners");
                }
                Log.i(TAG+"/inspectCorners", "Current ID is: " + ids.get(i, 0)[0]);
                Log.i(TAG+"/inspectCorners", "aruco_middle_x is: " + aruco_middle_x);
                Log.i(TAG+"/inspectCorners", "aruco_middle_z is: " + aruco_middle_z);

            }
            else {
                Log.i(TAG+"/inspectCorners", "Aruco ID: " + ids.get(i,0)[0] + " does not belong to current target: " + current_target);
            }
            Log.i(TAG+"/inspectCorners", "Current index is: " + i);
       }

        //eventual solution will be the aruco middle
        double[] aruco_middle = {aruco_middle_x, aruco_middle_z};
        return aruco_middle;
    }

    public Point moveCloserToArucoMarker(Kinematics kinematics, int current_target){

        double[] aruco_middle = inspectCorners(corners, current_target);// 5cm for each AR tags (ref to rulebook)

        int counter_x = 0;
        int counter_z = 0;
        final double middle_x = 1280/2;
        final double middle_z = 960/2;

        double aruco_middle_x = aruco_middle[0];
        double aruco_middle_z = aruco_middle[1];

        // Variable to detect the difference of 2 corners of 1 AR tag
        // arLength is the difference between 2 corners of the AR tag
        // use pythagoras for accuarcy
        double[] TopRightCoords = corners.get(0).get(0, 0);
        double[] BottomLeftCoords = corners.get(0).get(0, 2);
        double arLength = Math.sqrt(Math.pow((TopRightCoords[0] - BottomLeftCoords[0]), 2) + Math.pow((TopRightCoords[1] - BottomLeftCoords[1]), 2));
        Log.i(TAG+"/moveCloserToArucoMarker", "Length of AR Tag is: " + arLength);

        // scale is so that it can move as close as possible
        double scale = arLength / 5.0;

        double x_difference = middle_x - aruco_middle_x;
        Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is: " + x_difference);
        double z_difference = middle_z - aruco_middle_z;
        Log.i(TAG+"/moveCloserToArucoMarker", "The z difference is: " + z_difference);

        Point point;
        Point new_point;

        point = kinematics.getPosition();

        /* We need to use one of the AR tags as a reference of the bee moving closer to the image:
         * 1) We already have the coordinates of the corners (stored in Mat)
         * 2) Use one of the AR tag (while loop):
         *      - check the x difference of the corners of the 2 consecutive arrays (corners), difference stored as length
         *      - check for scale = length/5 (the exact length of the tag)
         * 3) New Point need to move according to scale (move as close as possible)
         */

        // initialize new_point
        new_point = new Point (point.getX(), point.getY(), point.getZ());

        /* We want the bee to move closer, thus each 4 points might be a bit different
         * Thought process:
         * 1) check for the x difference & z difference
         * 2) if the x diff & z diff still less than 20, repeat the image processing few times
          * */

        /* Case 1: positive difference case */
        while (x_difference >= scale){
            /* special case if x >= 100 */
            if (x_difference >= 100){
                new_point = new Point (point.getX() + (x_difference - 80), point.getY(), point.getZ());
                x_difference -= 80;
                //x_difference /= (move_relative / 2.0);
            }
            else {
                new_point = new Point (point.getX() + (x_difference - 20), point.getY(), point.getZ());
                x_difference -= 20;
                //x_difference /= (move_relative / 2.0);
            }
            counter_x ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is currently: " + x_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (x) attempted: " + counter_x);
        }

        while (z_difference >= scale){
            /* special case if z >= 100 */
            if (z_difference >= 100){
                new_point = new Point (point.getX(), point.getY(), point.getZ() + (z_difference - 80));
                z_difference -= 80;
                //z_difference *= (move_relative / 2.0);
            }
            else {
                new_point = new Point (point.getX(), point.getY(), point.getZ() + (z_difference - 20));
                z_difference -= 20;
               // z_difference *= (move_relative / 2.0);
            }
            counter_z ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The z difference is currently: " + z_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (z) attempted: " + counter_z);
        }

        /* Case 2: negative difference case */
        while (x_difference <= -scale){
            /* special case if x >= 100 */
            if (x_difference <= 100){
                new_point = new Point (point.getX() + (x_difference + 80), point.getY(), point.getZ());
                x_difference += 80;
                //x_difference /= (move_relative / 2.0);
            }
            else {
                new_point = new Point (point.getX() + (x_difference + 20), point.getY(), point.getZ());
                x_difference += 20;
                //x_difference /= (move_relative / 2.0);
            }
            counter_x ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The x difference is currently: " + x_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (x) attempted: " + counter_x);
        }

        while (z_difference <= -scale){
            /* special case if z >= 100 */
            if (z_difference <= 100){
                new_point = new Point (point.getX(), point.getY(), point.getZ() + (z_difference + 80));
                z_difference += 80;
               // z_difference /= (move_relative / 2.0);
            }
            else {
                new_point = new Point (point.getX(), point.getY(), point.getZ() + (z_difference + 20));
                z_difference += 20;
                //z_difference /= (move_relative / 2.0);
            }
            counter_z ++;
            Log.i(TAG+"/moveCloserToArucoMarker", "The z difference is currently: " + z_difference);
            Log.i(TAG+"/moveCloserToArucoMarker", "The new point is now: " + new_point);
            Log.i(TAG+"/moveCloserToArucoMarker", "The while (z) attempted: " + counter_z);
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