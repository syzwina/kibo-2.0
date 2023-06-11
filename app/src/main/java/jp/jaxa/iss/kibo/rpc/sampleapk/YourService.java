package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.os.Looper;
import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Board;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.aruco.Aruco;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();
    // initializing zones
    private final KeepOutZone KOZ01 = new KeepOutZone(9.8585f, -9.4500f, 4.82063f, 12.0085f, -8.5000f, 4.87063f);
    private final KeepOutZone KOZ02 = new KeepOutZone(9.8673f, -9.18813f, 3.81957f, 10.7673f, -8.28813f, 4.81957f);
    private final KeepOutZone KOZ03 = new KeepOutZone(11.1067f, -9.44819f, 4.87385f, 12.0067f, -8.89819f, 5.87385f);

    private final KeepInZone KIZ01 = new KeepInZone(10.3f, -10.2f, 4.32f, 11.55f, -6.4f, 5.57f);
    private final KeepInZone KIZ02 = new KeepInZone(9.5f, -10.5f, 4.02f, 10.5f, -9.6f, 4.8f);

    private int current_target = 0;


    @Override
    protected void runPlan1(){

        Log.i(TAG, "make dictionary");
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<Mat>();

        Mat ids = new Mat(1, 4, 1, new Scalar( 0, 150, 250 ));
        DetectorParameters detectorParameters = DetectorParameters.create();

        // the mission starts
        api.startMission();
        Log.i(TAG, "start mission!");

        current_target = 1;

        // POINT 1
        Point point = new Point(10.71f, -7.9f, 4.48f);
        // Point point = new Point(10.3f, -7.3f, 4.48f); tested with this, but failed
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
        moveBee(point, quaternion, 1);

        // report point1 arrival
        // api.reportPoint1Arrival();
        // new api doesn't have this?

        int counter = 0;
        while (counter < 10) {
            counter++;
            imageProcessing(dictionary, corners, detectorParameters, ids);
            moveCloserToArucoMarker(inspectCorners(corners));
            corners.clear();
        }



        // irradiate the laser
        Log.i(TAG, "turn laser on");
        api.laserControl(true);

        // take target1 snapshots
        Log.i(TAG, "take target 1 snapshot");
        api.takeTargetSnapshot(1);

        // turn the laser off
        Log.i(TAG, "turn laser off");
        api.laserControl(false);

        current_target = 2;

        // POINT 2 : lower than POINT 1
        point = new Point(10.7f, -7.7f, 5f);
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveBee(point, quaternion, 2);

        // POINT 3 : move forward towards target 2
        point = new Point(10.7f, -9.5f, 5f);
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveBee(point, quaternion, 3);

        // POINT 4 : move forward towards 'POINT 2 - as given in rulebook'
        point = new Point(11.27460f, -9.92284f, 5.29881f);
        // point = new Point(11.4f, -9.92284f, 5.29881f); tested with this but failed?
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveBee(point, quaternion, 4);



        counter = 0;
        while (counter < 10) {
            counter++;
            imageProcessing(dictionary, corners, detectorParameters, ids);
            moveCloserToArucoMarker(inspectCorners(corners));
            corners.clear();
        }



        // best not to turn lasers on when ydk if bee is pointing to the right thing

        // irradiate the laser
        Log.i(TAG, "turn laser on");
        api.laserControl(true);

        // take target2 snapshots
        api.takeTargetSnapshot(2);

        // turn the laser off
        Log.i(TAG, "turn laser off");
        api.laserControl(false);

        // POINT 5 : move back to previous position
        point = new Point(10.7f, -9.5f, 5f);
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveBee(point, quaternion, 5);

        // POINT 6 : go back to POINT 2
        point = new Point(10.7f, -7.7f, 5f);
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveBee(point, quaternion, 6);

        // POINT 7 : go to GOAL / CREW
        point = new Point(11.27460f, -7.89178f, 4.96538f);
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveBee(point, quaternion, 7);



        // send mission completion
        api.reportMissionCompletion("Mission Complete!");
        Log.i(TAG, "reported mission completion");

    }

    private boolean checksForKOZ(Point point){
        float x = (float) point.getX();
        float y = (float) point.getY();
        float z = (float) point.getZ();
        if (KOZ01.contains(x,y,z) || KOZ02.contains(x,y,z) || KOZ03.contains(x,y,x)) return false;
        return true;
    }

    private boolean checksForKIZ(Point point){
        float x = (float) point.getX();
        float y = (float) point.getY();
        float z = (float) point.getZ();
        if (KIZ01.contains(x,y,z) || KIZ02.contains(x,y,z)) return true;
        return false;
    }

    private void moveBee(Point point, Quaternion quaternion, int pointNumber){

        final int LOOP_MAX = 5;

        if (checksForKOZ(point)) Log.i(TAG, "point " + pointNumber + " is NOT in KOZ");
        else Log.e(TAG, "point " + pointNumber + " is in KOZ");
        if (checksForKIZ(point)) Log.i(TAG, "point " + pointNumber + " is in KIZ");
        else Log.e(TAG, "point " + pointNumber + " is NOT in KIZ");
        Log.i(TAG, "move to point " + pointNumber);
        Result result = api.moveTo(point, quaternion, false);

        // check result and loop while moveTo api is not succeeded
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            // retry
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
        if (result.hasSucceeded()) Log.i(TAG, "successfully moved to point " + pointNumber);
        else Log.e(TAG, "failed to move to point " + pointNumber);
        Log.i("coords", "point: x = " + point.getX() + ", y = " + point.getY() + ", z = " + point.getZ());
    }

    private double[] inspectCorners(List<Mat> corners) {

        Size size;
        double[] topright = {0,0};
        double[] topleft = {0,0};
        double[] bottomleft = {0,0};
        double[] bottomright = {0,0};
        final int x_coords = 0;
        final int y_coords = 1;

        for (int corner=0;corner<corners.size();corner++) {
            size = corners.get(corner).size();
            for (int j = 0; j < size.width; j++) {
                if (corner == 0 && j == 0) bottomleft = corners.get(corner).get(0, j);
                if (corner == 1 && j == 2) bottomright = corners.get(corner).get(0, j);
                if (corner == 2 && j == 0) topleft = corners.get(corner).get(0, j);
                if (corner == 3 && j == 2) topright = corners.get(corner).get(0, j);
            }
        }

        double aruco_middle_x = (bottomleft[x_coords] + bottomright[x_coords] + topleft[x_coords] + topright[x_coords])/4;
        double aruco_middle_y = (bottomleft[y_coords] + bottomright[y_coords] + topleft[y_coords] + topright[y_coords])/4;

        double[] aruco_middle = {aruco_middle_x, aruco_middle_y};

        return aruco_middle;
    }

    private void moveCloserToArucoMarker(double[] aruco_middle) {

        final double middle_x = 1280/2;
        final double middle_y = 960/2;

        double aruco_middle_x = aruco_middle[0];
        double aruco_middle_y = aruco_middle[1];

        double x_difference = middle_x - aruco_middle_x;
        double y_difference = middle_y - aruco_middle_y;

        Kinematics kinematics;
        Quaternion quaternion;
        Point point;
        Point new_point;

        if (current_target == 1) {

            kinematics = api.getRobotKinematics();
            quaternion = kinematics.getOrientation();
            point = kinematics.getPosition();

            if (x_difference < -50) {
                new_point = new Point(point.getX(), point.getY() + 0.2, point.getZ());
                moveBee(new_point, quaternion, 0); // move to right in y-axis
            }
            else if (x_difference > 50) {
                new_point = new Point(point.getX(), point.getY() - 0.2, point.getZ());
                moveBee(new_point, quaternion, 0); // move to left in y-axis
            }

            if (x_difference < -30) {
                new_point = new Point(point.getX(), point.getY() + 0.1, point.getZ());
                moveBee(new_point, quaternion, 0); // move to right in y-axis
            }
            else if (x_difference > 30) {
                new_point = new Point(point.getX(), point.getY() - 0.1, point.getZ());
                moveBee(new_point, quaternion, 0); // move to left in y-axis
            }

            if (x_difference < -20) {
                new_point = new Point(point.getX(), point.getY() + 0.05, point.getZ());
                moveBee(new_point, quaternion, 0); // move to right in y-axis
            }
            else if (x_difference > 20) {
                new_point = new Point(point.getX(), point.getY() - 0.05, point.getZ());
                moveBee(new_point, quaternion, 0); // move to left in y-axis
            }



            kinematics = api.getRobotKinematics();
            quaternion = kinematics.getOrientation();
            point = kinematics.getPosition();

            if (y_difference <  -50) {
                new_point = new Point(point.getX() + 0.2, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to down in x-axis
            }
            else if (y_difference > 50) {
                new_point = new Point(point.getX() - 0.2, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to up in x-axis
            }

            if (y_difference <  -30) {
                new_point = new Point(point.getX() + 0.1, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to down in x-axis
            }
            else if (y_difference > 30) {
                new_point = new Point(point.getX() - 0.1, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to up in x-axis
            }

            if (y_difference <  -20) {
                new_point = new Point(point.getX() + 0.05, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to down in x-axis
            }
            else if (y_difference > 20) {
                new_point = new Point(point.getX() - 0.05, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to up in x-axis
            }
        }


        if (current_target == 2) {

            kinematics = api.getRobotKinematics();
            quaternion = kinematics.getOrientation();
            point = kinematics.getPosition();

            if (x_difference < -50) {
                new_point = new Point(point.getX() + 0.2, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to right in x-axis
            }
            else if (x_difference > 50) {
                new_point = new Point(point.getX() - 0.2, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to left in x-axis
            }

            if (x_difference < -30) {
                new_point = new Point(point.getX() + 0.1, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to right in x-axis
            }
            else if (x_difference > 30) {
                new_point = new Point(point.getX() - 0.1, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to left in x-axis
            }

            if (x_difference < -20) {
                new_point = new Point(point.getX() + 0.05, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to right in x-axis
            }
            else if (x_difference > 20) {
                new_point = new Point(point.getX() - 0.05, point.getY(), point.getZ());
                moveBee(new_point, quaternion, 0); // move to left in x-axis
            }


            kinematics = api.getRobotKinematics();
            quaternion = kinematics.getOrientation();
            point = kinematics.getPosition();

            if (y_difference < -50) {
                new_point = new Point(point.getX(), point.getY(), point.getZ() + 0.1);
                moveBee(new_point, quaternion, 0); // move down in z-axis
            }
            else if (y_difference > 50) {
                new_point = new Point(point.getX(), point.getY(), point.getZ() - 0.1);
                moveBee(new_point, quaternion, 0); // move up in z-axis
            }

            if (y_difference < -30) {
                new_point = new Point(point.getX(), point.getY(), point.getZ() + 0.05);
                moveBee(new_point, quaternion, 0); // move down in z-axis
            }
            else if (y_difference > 30) {
                new_point = new Point(point.getX(), point.getY(), point.getZ() - 0.05);
                moveBee(new_point, quaternion, 0); // move up in z-axis
            }

            if (y_difference < -20) {
                new_point = new Point(point.getX(), point.getY(), point.getZ() + 0.025);
                moveBee(new_point, quaternion, 0); // move down in z-axis
            }
            else if (y_difference > 20) {
                new_point = new Point(point.getX(), point.getY(), point.getZ() - 0.025);
                moveBee(new_point, quaternion, 0); // move up in z-axis
            }
        }

    }

    private void imageProcessing(Dictionary dictionary, List<Mat> corners, DetectorParameters detectorParameters, Mat ids) {

        Mat image = api.getMatNavCam();
        image = api.getMatNavCam();
        api.saveMatImage(image, "nearTarget" + current_target + ".png");



        Log.i(TAG, "TARGET " + current_target + " image processing");
        Aruco.detectMarkers(image, dictionary, corners, ids, detectorParameters);
        Aruco.drawDetectedMarkers(image, corners, ids, new Scalar( 0, 150, 250 ));

        api.saveMatImage(image, "processedNearTarget" + current_target + ".png");

    }


}
