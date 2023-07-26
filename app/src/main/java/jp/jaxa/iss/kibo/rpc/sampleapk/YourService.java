package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.core.Scalar;
import org.opencv.aruco.DetectorParameters;

// not imported here due to naming conflicts
// but used explicitly
// import org.opencv.core.Point;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    private List<Integer> current_target;

    private int TIME_FOR_QR_AND_GOAL = 137 * 1000;

    // used globally as a way to know which point is the current goal
    private Point currentGoalCoords = new Point(0,0,0);
    private Quaternion currentQuaternion = new Quaternion(0,0,0,0);

    private int targetCounter = 0;

    private Dictionary dictionary;
    private List<Mat> corners;
    private Mat ids;
    private DetectorParameters detectorParameters;

    @Override
    protected void runPlan1(){

        dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        corners = new ArrayList<Mat>();
        ids = new Mat(1, 4, 1, new Scalar( 0, 150, 250 ));
        detectorParameters = DetectorParameters.create();

        // the mission starts
        api.startMission();
        Log.i(TAG+"/runPlan1", "start mission!");

        // move bee from KIZ2 to KIZ1 by moving to bottom right of KIZ2 (KIZ1 xyz min + KIZ2 xyz max)/2
        moveBee(new Point(10.4, -9.9, 4.50), PointConstants.POINT1_QUATERNION, 0);

        // count number of laser had been activated
        int laserCounter = 0;
        int phaseCounter = 0;

        // 4 phase
        while ( (phaseCounter < 7)) {

            targetCounter = 0;

            // add code to estimate if it is worth to go through with this phase
            // or should we skip immediately to last sequence
            //
            if (api.getTimeRemaining().get(1) < TIME_FOR_QR_AND_GOAL) {
                Log.e(TAG + "/runPlan1/OutOfTime", "BREAK SEQUENCE, TARGET COUNTER: " + targetCounter + ", TIME REMAINING: " + api.getTimeRemaining().get(1));
                lastSequence();
                break;
            }
            //
            //

            Log.i(TAG + "/runPlan1", "PHASE COUNTER = " + phaseCounter + ", TIME REMAINING:" + api.getTimeRemaining().get(1));
            phaseCounter++;

            current_target = api.getActiveTargets();
            int targetCounter = 0;
            Log.i(TAG+"/runPlan1", "ACTIVE TARGETS : " + current_target.toString());

            // while the number of targets visited is less than the number of active targets
            while (targetCounter < current_target.size()) {

                Log.i(TAG +"/runPlan1", "ACTIVE PHASE TIME BEFORE COMMON POINT MOVE: " + (api.getTimeRemaining().get(0)/1000) +" sec" );

                if (!moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter))) // -1 as index start at 0
                {// move bee to middle point of all points that not have KOZ on the way
                    Log.i(TAG + "/runPlan1/moveToCommon", "Attempt to move to next point directly is unsuccessful");
                    moveBee(PointConstants.COMMON_COORDS, PointConstants.POINT1_QUATERNION, 1000 + current_target.get(0));

                    // go to next phase if not enough time in current  phase (kinda illegal laser move lmao)
                    Log.i(TAG + "/runPlan1", "active phase time after common point move is: " + (api.getTimeRemaining().get(0) / 1000) + " seconds.");

                    // move bee to point 1
                    // instead of using 'target counter' to choose which point to take
                    // TODO: use a priority queue
                    moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter)); // -1 as index start at 0
                }

                // irradiate with laser
                laserBeam(current_target.get(targetCounter), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1));
                laserCounter++;
                Log.i(TAG + "/runPlan1/laserCounter", "LASER COUNTER: " + laserCounter);

                targetCounter++;
            }
        }

    }

    private void lastSequence(){

        // move bee to middle point of all points that not have KOZ on the way
        moveBee(PointConstants.COMMON_COORDS   , PointConstants.POINT7_QUATERNION, 1007);

        // move bee to target 7
        moveBee(PointConstants.POINT7_COORDS, PointConstants.POINT7_QUATERNION, 7);
        // turn on flashlight to improve accuracy, value taken from page 33 in manual
        api.flashlightControlFront(0.05f);
        // read QR code dummy function, not yet implemented
        String QRstring = readQR();
        // turn off flashlight
        api.flashlightControlFront(0);

        api.notifyGoingToGoal();

        // move to z axis of point 6 to avoid KOZ3
        moveBee(new Point(PointConstants.POINT7_COORDS.getX(),PointConstants.POINT7_COORDS.getY(), PointConstants.OLD_POINT6_COORDS.getZ()), PointConstants.GOAL_QUATERNION, 1008);

        moveBee(PointConstants.GOAL_COORDS, PointConstants.GOAL_QUATERNION, 8);

        // send mission completion
        api.reportMissionCompletion(QRstring);
        Log.i(TAG+"/runPlan1", "reported mission completion");

    }

    private void laserBeam(int current_target, Quaternion pointQuartenion){
        // compensate for laser pointer offset from navcam in plane that contains navcam/laser pointer by getting orientation first
        // yz axis offset value with local point origin at astrobee FROM NAVCAM under upright orientation ie x-axis as front to back axis (with front being
        // front view of astrobee in Figure 8-2
        double y_offset = -0.0994; // -0.0422 - 0.0572   //ie side/left/right offset
        double z_offset = 0.0285; // -0.0826 - (-0.1111) //ie up down offset
        // x in this case is front/back which not needed
        //currentQuaternion = api.getRobotKinematics().getOrientation();
        //probably need orientation check as well, cus now theres target in ceiling etc
        Log.i(TAG+"/laserBeam", "current Robot Position before offset compensation laser pointer: " + api.getRobotKinematics().getPosition().toString());

        if (current_target == 1) {
            api.relativeMoveTo(new Point(y_offset, 0, z_offset), pointQuartenion, true); //test target 1, +ve
        }
        if (current_target == 2){
            api.relativeMoveTo(new Point(y_offset, -z_offset, 0), pointQuartenion, true); //test target 2, correct wall now
        }
        if (current_target == 3) {
            api.relativeMoveTo(new Point(z_offset, y_offset, 0), pointQuartenion, true); //testing hypothesis on target 3, works!
        }
        if (current_target == 4) {
            api.relativeMoveTo(new Point(0, -y_offset, z_offset), pointQuartenion, true); //+ve y to go left,
        }
        if (current_target == 5) {
            api.relativeMoveTo(new Point(y_offset, z_offset, 0), pointQuartenion, true); // test
        }
        if (current_target == 6) {
            api.relativeMoveTo(new Point(0, y_offset, z_offset), pointQuartenion, true); // hopefully correct
        }
        Log.i(TAG+"/laserBeam", "current Robot Position after offset compensation laser pointer: " + api.getRobotKinematics().getPosition().toString());


        // turn on laser
        Log.i(TAG+"/laserBeam", "laser turned on");
        api.laserControl(true);

        // take laser image
/*        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, "LaserSnapshot" + current_target + ".png");*/

        api.takeTargetSnapshot(current_target);
    }


    private String readQR(){
        QRCodeMapper qrCodeMapper = new QRCodeMapper();
        String key = "";

        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, "QRImage.png");

        Log.i(TAG+"/readQR", "QR image processing");

        QRCodeReader qrCodeReader = new QRCodeReader();
        int qrCounter = 0;
        key = qrCodeReader.readQR(grayImage);
        while ((key.equals("NO QR")|| key.equals("")) && qrCounter < 5) {
        key = qrCodeReader.readQR(grayImage);
        Core.rotate(grayImage,grayImage, Core.ROTATE_90_CLOCKWISE);
        qrCounter++;
        Log.i(TAG+"/readQR", "QRCode key is: " + key + " attempt: " + qrCounter);
        }
        Log.i(TAG+"/readQR", "QRCode key is: " + key);
        return qrCodeMapper.getValue(key);
    }

    private boolean moveBee(Point point, Quaternion quaternion, int pointNumber){


        final int LOOP_MAX = 5;
        currentGoalCoords = point;
        currentQuaternion = quaternion;
        // probably not needed as all point is in KIZ
        /*if (checksForKOZ(point)) Log.i(TAG+"/moveBee", "point " + pointNumber + " is NOT in KOZ");
        else {
            Log.e(TAG+"/moveBee", "point " + pointNumber + " is in KOZ: " + currentKOZ.toString());
        }

        if (checksForKIZ(point)) Log.i(TAG+"/moveBee", "point " + pointNumber + " is in KIZ");
        else Log.e(TAG+"/moveBee", "point " + pointNumber + " is NOT in KIZ");*/
        Log.i(TAG+"/moveBee", "move to point " + pointNumber);
        Result result = api.moveTo(point, quaternion, true);
        Log.i(TAG+"/moveBee", "moveTo status:" + result.hasSucceeded());


        // check result and loop while moveTo api is not succeeded
        if (!result.hasSucceeded()) return false;

        if (result.hasSucceeded()) Log.i(TAG, "successfully moved to point " + pointNumber);
        else Log.e(TAG+"/moveBee", "failed to move to point " + pointNumber);
        Log.i(TAG+"/moveBee/coords", "point: x = " + point.getX() + ", y = " + point.getY() + ", z = " + point.getZ());
        return true;
    }


    // /**
    //  * Processes the provided image using ArUco marker detection and labeling.
    //  *
    //  * @param dictionary        The ArUco dictionary for marker detection.
    //  * @param corners           List to store the detected marker corners.
    //  * @param detectorParameters Parameters for the marker detector.
    //  * @param ids               Matrix to store the detected marker IDs.
    //  * @param targetID          The target ID for which the image is being processed.
    //  */
    // private void imageProcessing(Dictionary dictionary, List<Mat> corners, DetectorParameters detectorParameters, Mat ids, int targetID) {

    //     Mat grayImage = api.getMatNavCam();
    //     api.saveMatImage(grayImage, "Target_" + targetID + ".png");
    //     Log.i(TAG+"/imageProcessing", "Image has been saved in Black and White");

    //     Mat colorImage = new Mat();
    //     // Convert the grayscale image to color
    //     Imgproc.cvtColor(grayImage, colorImage, Imgproc.COLOR_GRAY2BGR);

    //     Log.i(TAG+"/imageProcessing", "TARGET " + targetID + " image processing");
    //     Aruco.detectMarkers(colorImage, dictionary, corners, ids, detectorParameters);
    //     Aruco.drawDetectedMarkers(colorImage, corners, ids, new Scalar( 0, 255, 0 ));

    //     Imgproc.putText(colorImage, "Aruco:"+ Arrays.toString(ids.get(0,0)), new org.opencv.core.Point(30.0, 80.0), 3, 0.5, new Scalar(255, 0, 0, 255), 1);
    //     Log.i(TAG+"imageProcessing", "Aruco marker has been labeled");

    //     api.saveMatImage(colorImage, "Processed_Target_" + targetID + ".png");
    //     Log.i(TAG+"imageProcessing", "Image has been saved in Colour");

    // }

    // private double[] inspectCorners(List<Mat> corners) {

    //     // once you choose one ID
    //     // decide which ID it is, and were it is relative to the centre of the circle
    //     // and set the new 'centre' coordinate to 'aruco_middle'

    //     // use mod 4 to get whether it is tl, tr, bl, br


    //     double[] topright;
    //     double[] topleft;
    //     double[] bottomleft;
    //     double[] bottomright;

    //     double aruco_middle_x = 0.0;
    //     double aruco_middle_y = 0.0;

    //     final int x_coords = 0;
    //     final int y_coords = 1;

    //     try{

    //     bottomleft  = corners.get(0).get(0, 2);
    //     bottomright = corners.get(0).get(0, 3);
    //     topleft     = corners.get(0).get(0, 1);
    //     topright    = corners.get(0).get(0, 0);

    //     aruco_middle_x = (bottomleft[x_coords] + bottomright[x_coords] + topleft[x_coords] + topright[x_coords])/4;
    //     aruco_middle_y = (bottomleft[y_coords] + bottomright[y_coords] + topleft[y_coords] + topright[y_coords])/4;
        
    //     }
    //     catch (Exception e) {
    //         e.printStackTrace();
    //     }

    //     double[] aruco_middle = {aruco_middle_x, aruco_middle_y};

    //     return aruco_middle;
    // }

    // private void optimizeCenter(int targetID){
    //     int img_process_counter = 0;
    //     while (img_process_counter < 2) {
    //         imageProcessing(dictionary, corners, detectorParameters, ids, targetID);
    //         // code to align astrobee with target 
    //         corners.clear();
    //         Log.i(TAG+"/optimizeCentre", "Optimizing Centre, attempt: " + img_process_counter);
    //         img_process_counter++;
    //     }
    // }

}
