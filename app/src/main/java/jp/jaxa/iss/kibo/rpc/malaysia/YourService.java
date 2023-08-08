package jp.jaxa.iss.kibo.rpc.malaysia;

import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Scalar;

import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;

import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    private List<Integer> current_target;

    private int TIME_FOR_QR_AND_GOAL = 151 * 1000;

    // used globally as a way to know which point is the current goal
    private Point currentGoalCoords = new Point(0,0,0);
    private Quaternion currentQuaternion = new Quaternion(0,0,0,0);

    private ImageProcessing imageProcessing;
    private QRCodeMapper qrCodeMapper = new QRCodeMapper();
    private QRCodeReader qrCodeReader = new QRCodeReader();

    private int targetCounter = 0;

    private Dictionary dictionary;
    private List<Mat> corners;
    private Mat ids;
    private DetectorParameters detectorParameters;

    private boolean QR_decoded = false;
    private String QRstring;

    private int called_image_save = 0;

    @Override
    protected void runPlan1(){

        dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        corners = new ArrayList<Mat>();
        ids = new Mat(1, 4, 1, new Scalar( 0, 150, 250 ));
        detectorParameters = DetectorParameters.create();

        imageProcessing = new ImageProcessing(dictionary, corners, ids, detectorParameters);


        // the mission starts
        api.startMission();
        Log.i(TAG+"/runPlan1", "start mission!");

        // TODO: can try to move to point directly first, if can't, then move to this point
        // move bee from KIZ2 to KIZ1 by moving to bottom right of KIZ2 (KIZ1 xyz min + KIZ2 xyz max)/2
        moveBee(new Point(10.4, -9.9, 4.50), PointConstants.POINT1_QUATERNION, 0);

        // count number of laser had been activated
        int laserCounter = 0;
        int phaseCounter = 0;

        // 4 phase
        while ( (phaseCounter < 7)) {

            targetCounter = 0;

            Log.i(TAG + "/runPlan1", "PHASE COUNTER = " + phaseCounter + ", TIME REMAINING:" + api.getTimeRemaining().get(1));
            phaseCounter++;
            
            current_target = api.getActiveTargets();
            Log.i(TAG+"/runPlan1", "ACTIVE TARGETS : " + current_target.toString());
            
            // add code to estimate if it is worth to go through with this phase
            // or should we skip immediately to last sequence
            if (api.getTimeRemaining().get(1) < TIME_FOR_QR_AND_GOAL) {
                Log.e(TAG + "/runPlan1/OutOfTime", "BREAK SEQUENCE, TARGET COUNTER: " + targetCounter + ", TIME REMAINING: " + api.getTimeRemaining().get(1));
                lastSequence();
                break;
            }
            
            // while the number of targets visited is less than the number of active targets
            while (targetCounter < current_target.size()) {

                Log.i(TAG +"/runPlan1", "ACTIVE PHASE TIME BEFORE COMMON POINT MOVE: " + (api.getTimeRemaining().get(0)/1000) +" sec" );

                if ((current_target.get(targetCounter) != 1) && !QR_decoded) {
                    Log.i(TAG +"/runPlan1", "READ QR EARLY");
                    readQrSequence();
                    QR_decoded = true;
                    TIME_FOR_QR_AND_GOAL -= 20 * 1000;
                }

                if (!moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter))) // -1 as index start at 0
                {
                    // move bee to middle point of all points that not have KOZ on the way
                    Log.i(TAG + "/runPlan1/moveToCommon", "UN-SUCCESSFUL ATTEMPT TO MOVE TO POINT DIRECTLY");
                    moveBee(PointConstants.COMMON_COORDS, PointConstants.POINT1_QUATERNION, current_target.get(0));

                    // TODO: use a priority queue
                    // move based on chosen target, targetCounter
                    moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter)); // -1 as index start at 0

                }
                else    Log.i(TAG + "/runPlan1/moveToCommon", "SUCCESSFUL ATTEMPT TO MOVE TO POINT DIRECTLY");

                // align astrobee to target
                Point new_point = optimizeCenter(current_target.get(targetCounter));

                // irradiate with laser
                laserBeam(current_target.get(targetCounter), new_point);

                Mat laserImage = api.getMatNavCam();
                Mat colorLaserImage = imageProcessing.imageProcessing(laserImage, current_target.get(targetCounter));
                api.saveMatImage(colorLaserImage, called_image_save + "_LaseredTarget_" + current_target + ".png");

                laserCounter++;
                Log.i(TAG + "/runPlan1/laserCounter", "LASER COUNTER: " + laserCounter);

                targetCounter++;
            }
        }

    }

    private void readQrSequence() {
        // move bee to middle point of all points that not have KOZ on the way
        Log.i(TAG + "/readQrSequence", "MOVE TO PRE-POINT OF QR");
        moveBee(PointConstants.COMMON_COORDS   , PointConstants.POINT7_QUATERNION, 7);

        // move bee to target 7
        Log.i(TAG + "/readQrSequence", "MOVE TO QR");
        moveBee(PointConstants.POINT7_COORDS, PointConstants.POINT7_QUATERNION, 7);

        // turn on flashlight to improve accuracy, value taken from page 33 in manual
        api.flashlightControlFront(0.05f);

        // read QR code
        QRstring = readQR();

        // turn off flashlight
        api.flashlightControlFront(0);
    }

    private void lastSequence(){

        if (!QR_decoded) readQrSequence();

        api.notifyGoingToGoal();

        // try to move to goal directly from current position
        if (!moveBee(PointConstants.GOAL_COORDS, PointConstants.GOAL_QUATERNION, 8))
        {
            // move to z axis of point 6 to avoid KOZ3
            Log.i(TAG + "/lastSequence", "MOVE TO AVOID KOZ3");
            if (!moveBee(new Point(PointConstants.POINT7_COORDS.getX(),PointConstants.POINT7_COORDS.getY(), PointConstants.OLD_POINT6_COORDS.getZ()), PointConstants.GOAL_QUATERNION, 1008))
            {
                // move to common point, needed if moving from target 2
                Log.i(TAG + "/lastSequence", "MOVE TO COMMON POINT");
                moveBee(PointConstants.COMMON_COORDS, PointConstants.POINT1_QUATERNION, current_target.get(0));
                Log.i(TAG + "/lastSequence", "MOVE TO AVOID KOZ3");
                moveBee(new Point(PointConstants.POINT7_COORDS.getX(),PointConstants.POINT7_COORDS.getY(), PointConstants.OLD_POINT6_COORDS.getZ()), PointConstants.GOAL_QUATERNION, 1008);
            }
            Log.i(TAG + "/lastSequence", "MOVE TO GOAL");
            moveBee(PointConstants.GOAL_COORDS, PointConstants.GOAL_QUATERNION, 8);
        }

        // send mission completion
        api.reportMissionCompletion(QRstring);
        Log.i(TAG + "/lastSequence", "REPORTED MISSION COMPLETED");

    }

    private void laserBeam(int current_target, Point optimized_point){
        // compensate for laser pointer offset from navcam in plane that contains navcam/laser pointer by getting orientation first
        // yz axis offset value with local point origin at astrobee FROM NAVCAM under upright orientation ie x-axis as front to back axis (with front being
        // front view of astrobee in Figure 8-2
        double y_offset = 0.5; // -0.0422 - 0.0572   //ie side/left/right offset
        double z_offset = 0.5; // -0.0826 - (-0.1111) //ie up down offset
        Log.i(TAG+"/laserBeam", "y_offset is: " + y_offset);
        Log.i(TAG+"/laserBeam", "z_offset is: " + z_offset);
        // x in this case is front/back which not needed
        //currentQuaternion = api.getRobotKinematics().getOrientation();
        //probably need orientation check as well, cus now theres target in ceiling etc
        Log.i(TAG+"/laserBeam", "current Robot Position before offset compensation laser pointer: " + api.getRobotKinematics().getPosition().toString());

        Point new_point;

        if (current_target == 1) {
            new_point = new Point(optimized_point.getX() + y_offset, optimized_point.getY(), optimized_point.getZ() + z_offset); //test target 1, +ve
            Log.i(TAG+"/laserBeam", "USE TARGET: 1");
        }
        else if (current_target == 2){
            new_point = new Point(optimized_point.getX() + y_offset, optimized_point.getY() - z_offset, optimized_point.getZ()); //test target 2, correct wall now
            Log.i(TAG+"/laserBeam", "USE TARGET: 2");
        }
        else if (current_target == 3) {
            new_point = new Point(optimized_point.getX() + z_offset, optimized_point.getY() + y_offset, optimized_point.getZ()); //testing hypothesis on target 3, works!
            Log.i(TAG+"/laserBeam", "USE TARGET: 3");
        }
        else if (current_target == 4) {
            new_point = new Point(optimized_point.getX(), optimized_point.getY()- y_offset, optimized_point.getZ() + z_offset); //+ve y to go left,
            Log.i(TAG+"/laserBeam", "USE TARGET: 4");
        }
        else {
            new_point = new Point(0,0,0);
            Log.i(TAG+"/laserBeam", "ALERT: Unknown Target " + current_target);
        }
        Log.i(TAG+"/laserBeam", "FINAL CALCULATED POINT: " + new_point);

        api.relativeMoveTo(new_point, PointConstants.POINTS_QUATERNIONS.get(this.current_target.get(targetCounter) - 1), false);

        Log.i(TAG+"/laserBeam", "current Robot Position after offset compensation laser pointer: " + api.getRobotKinematics().getPosition().toString());


        // turn on laser
        Log.i(TAG+"/laserBeam", "laser turned on");
        api.laserControl(true);

        api.takeTargetSnapshot(current_target);
    }


    private String readQR(){
        called_image_save++;

        Log.i(TAG+"/readQR", "QR IMAGE PROCESSING");
        
        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, called_image_save + "_QRImage.png");
        
        String key = "";
        key = qrCodeReader.readQR(grayImage);
        
        // loop and turn the image multiple times to get the QR key
        int qrCounter = 0;
        while ((key.equals("NO QR")|| key.equals("")) && qrCounter < 5) 
        {
            qrCounter++;
            key = qrCodeReader.readQR(grayImage);
            Core.rotate(grayImage, grayImage, Core.ROTATE_90_CLOCKWISE);
            api.saveMatImage(grayImage, called_image_save + "_" + qrCounter + "_QRImage.png");
            Log.i(TAG+"/readQR", "QRCODE KEY: " + key + " ATTEMPT: " + qrCounter);
        }
        Log.i(TAG+"/readQR", "QRCODE KEY: " + key);

        return qrCodeMapper.getValue(key);
    }

    private boolean moveBee(Point point, Quaternion quaternion, int pointNumber){

        currentGoalCoords = point;
        currentQuaternion = quaternion;

        // probably not needed as all point is in KIZ
        /*if (checksForKOZ(point)) Log.i(TAG+"/moveBee", "point " + pointNumber + " is NOT in KOZ");
        else {
            Log.e(TAG+"/moveBee", "point " + pointNumber + " is in KOZ: " + currentKOZ.toString());
        }

        if (checksForKIZ(point)) Log.i(TAG+"/moveBee", "point " + pointNumber + " is in KIZ");
        else Log.e(TAG+"/moveBee", "point " + pointNumber + " is NOT in KIZ");*/

        Log.i(TAG+"/moveBee", "MOVE TO: " + pointNumber);
        Result result = api.moveTo(point, quaternion, false);

        if (result.hasSucceeded())  
        {
            Log.i(TAG + "/moveBee", "MOVETO STATUS: SUCCESS");
            Log.i(TAG + "/moveBee", "SUCCESSFULLY MOVED TO POINT: " + pointNumber);
            // TODO: print the actual current point and quaternion using kinematics
            // beacuse it might not go exactly to the point
            Log.i(TAG + "/moveBee/coords", "X: " + point.getX() + ", Y: " + point.getY() + ", Z: " + point.getZ());
            return true;
        }
        else 
        {
            Log.e(TAG + "/moveBee", "MOVETO STATUS: FAILED");
            Log.e(TAG + "/moveBee", "UN-SUCCESSFULLY MOVED TO POINT: " + pointNumber);
            return false;
        }

    }

    public Point optimizeCenter(int targetID){
        called_image_save++;

        Log.i(TAG+"/optimizeCenter", "OPTIMIZING CENTER");
        // image processing to figure our position of target
        Mat grayImage = api.getMatNavCam();
        Mat colorImage = imageProcessing.imageProcessing(grayImage, targetID);
        api.saveMatImage(colorImage, called_image_save + "_Target_" + targetID + ".png");

        // code to align astrobee with target
        Point new_point = imageProcessing.moveCloserToArucoMarker(targetID);

        imageProcessing.corners.clear();

        return new_point;
        }
}
