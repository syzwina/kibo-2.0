package jp.jaxa.iss.kibo.rpc.sampleapk;

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

import java.util.List;
import java.util.ArrayList;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    private List<Integer> current_target;

    private int TIME_FOR_QR_AND_GOAL = 150 * 1000;

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

                // TODO: check to go to QR first, when you're in the area
                if (!current_target.contains(1) && !QR_decoded) {
                    Log.i(TAG +"/runPlan1", "READ QR EARLY");
                    readQrSequence();
                    QR_decoded = true;
                }

                if (!moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter))) // -1 as index start at 0
                {
                    // move bee to middle point of all points that not have KOZ on the way
                    Log.i(TAG + "/runPlan1/moveToCommon", "UN-SUCCESSFUL ATTEMPT TO MOVE TO POINT DIRECTLY");
                    moveBee(PointConstants.COMMON_COORDS, PointConstants.POINT1_QUATERNION, current_target.get(0));

                    // move bee to point 1
                    // instead of using 'target counter' to choose which point to take
                    // TODO: use a priority queue
                    moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter)); // -1 as index start at 0

                    // align astrobee to target
                    optimizeCenter(current_target.get(targetCounter));
                }
                else 
                {
                    Log.i(TAG + "/runPlan1/moveToCommon", "SUCCESSFUL ATTEMPT TO MOVE TO POINT DIRECTLY");

                    // align astrobee to target
                    optimizeCenter(current_target.get(targetCounter));
                }

                // irradiate with laser
                laserBeam(current_target.get(targetCounter), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1));
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

        // move to z axis of point 6 to avoid KOZ3
        Log.i(TAG + "/lastSequence", "MOVE TO AVOID KOZ3");
        moveBee(new Point(PointConstants.POINT7_COORDS.getX(),PointConstants.POINT7_COORDS.getY(), PointConstants.OLD_POINT6_COORDS.getZ()), PointConstants.GOAL_QUATERNION, 1008);

        Log.i(TAG + "/lastSequence", "MOVE TO GOAL");
        moveBee(PointConstants.GOAL_COORDS, PointConstants.GOAL_QUATERNION, 8);

        // send mission completion
        api.reportMissionCompletion(QRstring);
        Log.i(TAG + "/lastSequence", "REPORTED MISSION COMPLETED");

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

        api.takeTargetSnapshot(current_target);
    }


    private String readQR(){

        Log.i(TAG+"/readQR", "QR IMAGE PROCESSING");
        
        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, "QRImage.png");
        
        String key = "";
        key = qrCodeReader.readQR(grayImage);
        
        // loop and turn the image multiple times to get the QR key
        int qrCounter = 0;
        while ((key.equals("NO QR")|| key.equals("")) && qrCounter < 5) 
        {
            qrCounter++;
            key = qrCodeReader.readQR(grayImage);
            Core.rotate(grayImage, grayImage, Core.ROTATE_90_CLOCKWISE);
            api.saveMatImage(grayImage, "QRImage_" + qrCounter + ".png");
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
        Result result = api.moveTo(point, quaternion, true);

        if (result.hasSucceeded())  
        {
            Log.i(TAG + "/moveBee", "MOVETO STATUS: SUCCESS");
            Log.i(TAG + "/moveBee", "SUCCESSFULLY MOVED TO POINT: " + pointNumber);
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

    public void optimizeCenter(int targetID){
        Log.i(TAG+"/optimizeCenter", "OPTIMIZING CENTER");
        // image processing to figure our position of target
        Mat grayImage = api.getMatNavCam();
        Mat colorImage = imageProcessing.imageProcessing(grayImage, targetID);
        api.saveMatImage(colorImage, "ProcessedTarget_" + targetID + ".png");

        // code to align astrobee with target
        Kinematics kinematics = api.getRobotKinematics();
        Point new_point = imageProcessing.moveCloserToArucoMarker(kinematics, imageProcessing.inspectCorners(imageProcessing.corners), targetID);
        api.moveTo(new_point, kinematics.getOrientation(), true);
        Mat alignedImage = api.getMatNavCam();
        Mat colorAlignedImage = imageProcessing.imageProcessing(alignedImage, targetID);
        api.saveMatImage(colorAlignedImage, "AlignedProcessedTarget_" + targetID + ".png");
        imageProcessing.corners.clear();
        }
}
