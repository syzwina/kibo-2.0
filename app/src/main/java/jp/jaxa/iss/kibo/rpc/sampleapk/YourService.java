package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.Core;

// not imported here due to naming conflicts
// but used explicitly
// import org.opencv.core.Point;

import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    private List<Integer> current_target;

    private int TIME_FOR_QR_AND_GOAL = 122 * 1000;

    // used globally as a way to know which point is the current goal
    private Point currentGoalCoords = new Point(0,0,0);
    private Quaternion currentQuaternion = new Quaternion(0,0,0,0);

    @Override
    protected void runPlan1(){

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
            Log.i(TAG + "/runPlan1", "at start of Phase counter = " + phaseCounter + ", TIME REMAINING:" + api.getTimeRemaining().get(1));
            phaseCounter++;

            current_target = api.getActiveTargets();
            int targetCounter = 0;
            Log.i(TAG+"/runPlan1", "getting active targets which are : " + current_target.toString());

            while (targetCounter < current_target.size()) {

                Log.i(TAG +"/runPlan1", "active phase time before common point move is: " + (api.getTimeRemaining().get(0)/1000) +" seconds." );


                if (laserCounter == 3) {
                    TIME_FOR_QR_AND_GOAL += 10 * 1000; // at 3rd target, increase time taken
                }

                if (api.getTimeRemaining().get(1) < TIME_FOR_QR_AND_GOAL + 10 *1000) {
                    Log.e(TAG + "/runPlan1/OutOfTime", "Sequence1 broken at targetCounter of " + targetCounter + " as not enough time, TIME REMAINING: " + api.getTimeRemaining().get(1));
                    lastSequence();
                    break;
                }

                if (!moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter))) // -1 as index start at 0
                {// move bee to middle point of all points that not have KOZ on the way
                    Log.i(TAG + "/runPlan1/moveToCommon", "Attempt to move to next point directly is unsuccessful");
                    moveBee(PointConstants.COMMON_COORDS, PointConstants.POINT1_QUATERNION, 1000 + current_target.get(0));

                    // go to next phase if not enough time in current  phase (kinda illegal laser move lmao)
                    Log.i(TAG + "/runPlan1", "active phase time after common point move is: " + (api.getTimeRemaining().get(0) / 1000) + " seconds.");

                    if (api.getTimeRemaining().get(1) < TIME_FOR_QR_AND_GOAL + 5 * 1000) {
                        Log.e(TAG + "/runPlan1/OutOfTime", "Sequence2 broken at targetCounter of " + targetCounter + " as not enough time, TIME REMAINING: " + api.getTimeRemaining().get(1));
                        lastSequence();
                        break;
                    }

                    Log.i(TAG + "/runPlan1", "before going to point = " + current_target.get(0) + ", TIME REMAINING:" + api.getTimeRemaining().get(1));
                    // move bee to point 1
                    moveBee(PointConstants.POINTS_COORDS.get(current_target.get(targetCounter) - 1), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1), current_target.get(targetCounter)); // -1 as index start at 0
                }


                // turn on flashlight to improve accuracy, value taken from page 33 in manual
                // api.flashlightControlFront(0.05f); //not really needed
                // optimize center using image processing the corners
                //optimizeCenter(current_target.get(targetCounter));
                // to reset active id ??
                api.getActiveTargets();
                // irradiate with laser
                laserBeam(current_target.get(targetCounter), PointConstants.POINTS_QUATERNIONS.get(current_target.get(targetCounter) - 1));
                laserCounter++;
                Log.i(TAG+"/runPlan1/laserCounter", "laserCounter value is: " + laserCounter);
                // turn off flashlight
                // api.flashlightControlFront((float) 0);
                Log.i(TAG + "/runPlan1", "current_target after laser beam count: " + laserCounter + " are: " + current_target);
                Log.i(TAG + "/runPlan1", "getActiveTargets return:" + api.getActiveTargets().toString());


                if (api.getTimeRemaining().get(1) < TIME_FOR_QR_AND_GOAL) {
                    Log.e(TAG + "/runPlan1/OutOfTime", "Sequence3 broken at targetCounter of " + targetCounter + " as not enough time, TIME REMAINING: " + api.getTimeRemaining().get(1));
                    lastSequence();
                    break;
                }

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

}
