package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.aruco.Aruco;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
//import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

// for pathfinding
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();
    // initializing zones
    private final KeepOutZone KOZ01 = new KeepOutZone(10.783f, -9.8899f, 4.8385f, 11.071f, -9.6929f, 5.0665f);
    private final KeepOutZone KOZ02 = new KeepOutZone(10.8652f, -9.0734f, 4.3861f, 10.9628f, -8.7314f, 4.6401f);
    private final KeepOutZone KOZ03 = new KeepOutZone(10.185f, -8.3826f, 4.1475f, 11.665f, -8.2826f, 4.6725f);
    private final KeepOutZone KOZ04 = new KeepOutZone(10.7955f, -8.0635f, 5.1055f, 11.3525f, -7.7305f, 5.1305f);
    private final KeepOutZone KOZ05 = new KeepOutZone(10.563f, -7.1449f, 4.6544f, 10.709f, -6.8099f, 4.8164f);
    private KeepOutZone currentKOZ = new KeepOutZone(0f,0f,0f,0f,0f,0f); // not really needed but put it here anyway

    private final KeepInZone KIZ01 = new KeepInZone(10.3f, -10.2f, 4.32f, 11.55f, -6.0f, 5.57f);
    private final KeepInZone KIZ02 = new KeepInZone(9.5f, -10.5f, 4.02f, 10.5f, -9.6f, 4.8f);

    private List<Integer> current_target;


    /**
     * Constants defined from RULEBOOK
     */

    private Point currentCoords = new Point(0,0,0);
    private Point currentGoalCoords = new Point(0,0,0);
    private final Point START_COORDS = new Point(9.815, -9.806, 4.293);
    private final Point GOAL_COORDS = new Point(11.143, -6.7607, 4.9654);
    private final Point POINT1_COORDS = new Point(11.2746, -9.92284, 5.2988);
    private final Point POINT2_COORDS = new Point(10.612, -9.0709, 4.48);
    private final Point POINT3_COORDS = new Point(10.71, -7.7, 4.48);
    private final Point POINT4_COORDS = new Point(10.51, -6.7185, 5.1804);
    private final Point POINT5_COORDS = new Point(11.114, -7.9756, 5.3393);
    private final Point POINT6_COORDS = new Point(11.355, -8.9929, 4.7818);
    private final Point POINT7_COORDS = new Point(11.369, -8.5518, 4.48);
    List<Point> POINTS_COORDS = Arrays.asList(POINT1_COORDS, POINT2_COORDS, POINT3_COORDS,
            POINT4_COORDS, POINT5_COORDS, POINT6_COORDS, POINT7_COORDS);

    private final Point TARGET1_COORDS = new Point(11.2625, -10.58, 5.3625);
    private final Point TARGET2_COORDS = new Point(10.513384, -9.085172, 3.76203);
    private final Point TARGET3_COORDS = new Point(10.6031, -7.71007, 3.76093);
    private final Point TARGET4_COORDS = new Point(9.866984, -6.673972, 5.09531);
    private final Point TARGET5_COORDS = new Point(11.102, -8.0304, 5.9076);
    private final Point TARGET6_COORDS = new Point(12.023, -8.989, 4.8305);
    private final Point QR_CODE_COORDS = new Point(11.381944, -8.566172, 3.76203);

    private Quaternion currentQuaternion = new Quaternion(0,0,0,0);
    private final Quaternion START_QUATERNION = new Quaternion((float) 1, (float) 0, (float) 0, (float) 0);
    private final Quaternion GOAL_QUATERNION = new Quaternion((float) 0, (float) 0, (float) -0.707, (float) 0.707);
    private final Quaternion POINT1_QUATERNION = new Quaternion((float) 0, (float) 0, (float) -0.707, (float) 0.707);
    private final Quaternion POINT2_QUATERNION = new Quaternion((float) 0.5, (float) 0.5, (float) -0.5, (float) 0.5);
    private final Quaternion POINT3_QUATERNION = new Quaternion((float) 0, (float) 0.707, (float) 0, (float) 0.707);
    private final Quaternion POINT4_QUATERNION = new Quaternion((float) 0, (float) 0, (float) -1, (float) 0);
    private final Quaternion POINT5_QUATERNION = new Quaternion((float) -0.5, (float) -0.5, (float) -0.5, (float) 0.5);
    private final Quaternion POINT6_QUATERNION = new Quaternion((float) 0, (float) 0, (float) 0, (float) 1);
    private final Quaternion POINT7_QUATERNION = new Quaternion((float) 0, (float) 0.707, (float) 0, (float) 0.707);
    List<Quaternion> POINTS_QUARTENIONS = Arrays.asList(POINT1_QUATERNION, POINT2_QUATERNION, POINT3_QUATERNION,
            POINT4_QUATERNION, POINT5_QUATERNION, POINT6_QUATERNION, POINT7_QUATERNION);

    private final Quaternion TARGET1_QUATERNION = new Quaternion((float) 0.707, (float) 0, (float) 0, (float) 0.707);
    private final Quaternion TARGET2_QUATERNION = new Quaternion((float) 0, (float) 0, (float) 0, (float) 1);
    private final Quaternion TARGET3_QUATERNION = new Quaternion((float) 0.707, (float) 0, (float) 0, (float) 0.707);
    private final Quaternion TARGET4_QUATERNION = new Quaternion((float) -0.5, (float) 0.5, (float) -0.5, (float) 0.5);
    private final Quaternion TARGET5_QUATERNION = new Quaternion((float) 1, (float) 0, (float) 0, (float) 0);
    private final Quaternion TARGET6_QUATERNION = new Quaternion((float) 0.5, (float) 0.5, (float) -0.5, (float) -0.5);
    private final Quaternion QR_CODE_QUATERNION = new Quaternion((float) 0, (float) 0, (float) 0, (float) 1);

    HashMap<Integer, Integer> arucoTargets;
    DetectorParameters detectorParameters;
    List<Mat> corners;
    Dictionary dictionary;
    Mat ids;


    @Override
    protected void runPlan1(){

        init();

        // the mission starts
        api.startMission();
        Log.i(TAG, "start mission!");

        // move bee from KIZ2 to KIZ1 by moving to bottom right of KIZ2 (KIZ1 xyz min + KIZ2 xyz max)/2
        moveBee(new Point(10.4, -9.9, 4.50), POINT1_QUATERNION, 0);
        int counter = 0;
        // 4 phase
        while ( (counter < 4) && (api.getTimeRemaining().get(1) > 120 * 1000) ) {
            Log.i(TAG, "TIME REMAINING:" + api.getTimeRemaining().get(1));
            counter++;

            current_target = api.getActiveTargets();
            Log.i("target acquisition", "getting active targets which are : " + current_target.toString());


            // move bee to point 1
            moveBee(POINTS_COORDS.get(current_target.get(0)-1), POINTS_QUARTENIONS.get(current_target.get(0)-1), current_target.get(0)); // -1 as index start at 0
            // turn on flashlight to improve accuracy, value taken from page 33 in manual
            api.flashlightControlFront((float) 0.5);
            // optimize center using image processing the corners
            optimizeCenter(current_target.get(0));
            // irradiate with laser
            laserBeam(current_target.get(0));
            // turn off flashlight
            api.flashlightControlFront((float) 0);


            // check if there's second point in one phase
            if (current_target.size() >= 2) {

                // check if phase still on going
                if (current_target.get(0) == api.getActiveTargets().get(0)) {

                    // move bee to point 2
                    moveBee(POINTS_COORDS.get(current_target.get(1) - 1), POINTS_QUARTENIONS.get(current_target.get(1) - 1), current_target.get(1)); // -1 as index start at 0
                    // turn on flashlight to improve accuracy, value taken from page 33 in manual
                    api.flashlightControlFront((float) 0.5);
                    // optimize center using image processing the corners
                    optimizeCenter(current_target.get(1));
                    // irradiate with laser
                    laserBeam(current_target.get(1));
                    // turn off flashlight
                    api.flashlightControlFront((float) 0);
                }
            }

        }


        // take target1 snapshots
//        Log.i(TAG, "take target 1 snapshot");
//        api.takeTargetSnapshot(1);

        // move bee to target 7
/*        moveBee(POINT7_COORDS, POINT7_QUATERNION, 7);
        // turn on flashlight to improve accuracy, value taken from page 33 in manual
        api.flashlightControlFront( (float) 0.5);
        // optimize center using image processing the corners
        optimizeCenter(7);
        // read QR code dummy function, not yet implemented
        readQR();
        // turn off flashlight
        api.flashlightControlFront((float) 0);*/

        api.notifyGoingToGoal();
        moveBee(GOAL_COORDS, GOAL_QUATERNION, 8);

        // send mission completion
        api.reportMissionCompletion("Mission Complete!");
        Log.i(TAG, "reported mission completion");

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

    private void optimizeCenter(int targetID){
        int img_process_counter = 0;
        while (img_process_counter < 2) {
            imageProcessing(dictionary, corners, detectorParameters, ids, targetID);
            moveCloserToArucoMarker(inspectCorners(corners), targetID);
            corners.clear();
            img_process_counter++;
        }
    }

    private void laserBeam(int current_target){
        // compensate for laser pointer offset from navcam in plane that contains navcam/laser pointer by getting orientation first
        // yz axis offset value with local point origin at astrobee FROM NAVCAM under upright orientation ie x-axis as front to back axis (with front being
        // front view of astrobee in Figure 8-2
        double y_offset = -0.0422 - 0.0572;
        double z_offset = -0.0826 - (-0.1111);
        currentQuaternion = api.getRobotKinematics().getOrientation();
        Log.i("laserBeam", "current Robot Position before offset compensation laser pointer: " + api.getRobotKinematics().getPosition().toString());
        api.relativeMoveTo(new Point(0, y_offset,z_offset), currentQuaternion, true);
        Log.i("laserBeam", "current Robot Position after offset compensation laser pointer: " + api.getRobotKinematics().getPosition().toString());

        //commented for now as not known needed or not
/*        if (cubeOrientation(currentQuaternion) == 'x'){
            // move relative to yz axis
            api.relativeMoveTo()
        }
        else if (cubeOrientation(currentQuaternion) == 'y'){
            // move relative to xz axis

        }
        else if (cubeOrientation(currentQuaternion) == 'z'){
            // move relative to xy axis

        }
        else {
            Log.e("laserBeam", "cubeOrientation is fked up");
        }*/
        // turn on laser
        Log.i("laserBeam", "laser turned on");
        api.laserControl(true);

        // take laser image
        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, "LaserSnapshotBW" + current_target + ".png");
        Mat colorImage = new Mat();
        // Convert the grayscale image to color
        Imgproc.cvtColor(grayImage, colorImage, Imgproc.COLOR_GRAY2BGR);
        api.saveMatImage(colorImage, "LaserSnapshotImage" + current_target + ".png");

        api.takeTargetSnapshot(current_target);

    }

    // not sure needed or not
    private char cubeOrientation(Quaternion quaternion){
                // quaternion values
                double x = quaternion.getX();
                double y = quaternion.getY();
                double z = quaternion.getZ();
                double w = quaternion.getW();

                // Determine the axis the cube is facing
                double largestComponent = Math.max(Math.abs(x), Math.max(Math.abs(y), Math.abs(z)));
                if (largestComponent == Math.abs(x)) {
                    Log.i("cubeOrientation", "Cube face is on x axis");
                    return 'x';
                } else if (largestComponent == Math.abs(y)) {
                    Log.i("cubeOrientation", "Cube face is on y axis");
                    return 'y';
                } else if (largestComponent == Math.abs(z)){
                    Log.i("cubeOrientation", "Cube face is on z axis");
                    return 'z';
                }
                else {
                    Log.e("cubeOrientation", "Cube face is facing somewhere not aligned to axes???");
                    return 'F';
                }
    }

    private void readQR(){

    }

    private boolean checksForKOZ(Point point){
        float x = (float) point.getX();
        float y = (float) point.getY();
        float z = (float) point.getZ();
        if (KOZ01.contains(x,y,z)) {
            currentKOZ = KOZ01;
            return false;
        }
        else if (KOZ02.contains(x,y,z)){
            currentKOZ = KOZ02;
            return false;
        }
        else if (KOZ03.contains(x,y,z)){
            currentKOZ = KOZ03;
            return false;
        }
        else if (KOZ04.contains(x,y,z)){
            currentKOZ = KOZ04;
            return false;
        }
        else if (KOZ05.contains(x,y,z)){
            currentKOZ = KOZ05;
            return false;
        }
        else return true;
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
        currentGoalCoords = point;
        currentQuaternion = quaternion;
        if (checksForKOZ(point)) Log.i(TAG, "point " + pointNumber + " is NOT in KOZ");
        else {
            Log.e("moveBee", "point " + pointNumber + " is in KOZ: " + currentKOZ.toString());
            pathfind();
        }
        if (checksForKIZ(point)) Log.i(TAG, "point " + pointNumber + " is in KIZ");
        else Log.e("moveBee", "point " + pointNumber + " is NOT in KIZ");
        Log.i("moveBee", "move to point " + pointNumber);
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

    private void pathfind(){
        Log.i("pathfind", "Pathfinding activated");
        if (api.getRobotKinematics().getConfidence() == Kinematics.Confidence.GOOD) {
            currentCoords = api.getRobotKinematics().getPosition();
            Log.i("pathfind", "current coords is: x = " + currentCoords.getX() + ", y = " + currentCoords.getY() + ", z = " + currentCoords.getZ());
            Quaternion currentOrientation = api.getRobotKinematics().getOrientation();
            Log.i("pathfind", "current orient is: w = " + currentOrientation.getW() + " x = " + currentOrientation.getX() + ", y = " + currentOrientation.getY() + ", z = " + currentOrientation.getZ());
            //try move out of corresponding KOZ

            Vector3D currentPosition = new Vector3D(currentCoords.getX(),currentCoords.getY(),currentCoords.getZ());
            Vector3D goalPosition = new Vector3D(currentGoalCoords.getX(),currentGoalCoords.getY(),currentGoalCoords.getZ());
            while (!reachedGoal(currentPosition, goalPosition)){
                int obstructedAxis = findObstructedAxis(currentPosition, goalPosition);

                if (obstructedAxis == -1) {
                    // No obstructed axis, move directly towards the goal
                    moveTowardsGoal(currentPosition,goalPosition);
                } else {
                    // Move along the axis with the least obstruction
                    moveAlongAxis(obstructedAxis,currentPosition,goalPosition);
                }
            }
        }
    }

    private boolean reachedGoal(Vector3D currentPosition, Vector3D goalPosition) {
        // Check if the current position is close enough to the goal
        double threshold = 0.1;
        double distance = currentPosition.distance(goalPosition);

        return distance < threshold;
    }

    private int findObstructedAxis(Vector3D currentPosition, Vector3D goalPosition) {
        // Check for obstructions along each axis (X, Y, Z)
        // Determine which axis has the least obstruction
        // In this example, assume the axis with the smallest difference in position is the least obstructed

        double dx = Math.abs(currentPosition.getX() - goalPosition.getX());
        double dy = Math.abs(currentPosition.getY() - goalPosition.getY());
        double dz = Math.abs(currentPosition.getZ() - goalPosition.getZ());

        if (dx < dy && dx < dz) {
            return 0; // X-axis is obstructed
        } else if (dy < dx && dy < dz) {
            return 1; // Y-axis is obstructed
        } else if (dz < dx && dz < dy) {
            return 2; // Z-axis is obstructed
        } else {
            return -1; // No axis is obstructed or they are obstructed equally
        }
    }

    private void moveTowardsGoal(Vector3D currentPosition, Vector3D goalPosition) {
        // Move towards the goal by a certain distance or step size
        double stepSize = 0.1;

        Vector3D direction = goalPosition.subtract(currentPosition);
        double distance = direction.getNorm();
        double ratio = stepSize / distance;

        Vector3D newPosition = currentPosition.add(direction.scalarMultiply(ratio));
        //currentPosition = newPosition;
        Point newPoint = new Point(newPosition.getX(),newPosition.getY(),newPosition.getZ());
        moveBee(newPoint, currentQuaternion, 666); // random number?
        System.out.println("Moved towards goal: " + newPosition);
    }

    private void moveAlongAxis(int axis, Vector3D currentPosition, Vector3D goalPosition) {
        // Move along the specified axis
        double stepSize = 0.1;

        Vector3D direction;
        if (axis == 0) {
            // Move along the X-axis
            direction = new Vector3D(goalPosition.getX() - currentPosition.getX(), 0, 0);
        } else if (axis == 1) {
            // Move along the Y-axis
            direction = new Vector3D(0, goalPosition.getY() - currentPosition.getY(), 0);
        } else {
            // Move along the Z-axis
            direction = new Vector3D(0, 0, goalPosition.getZ() - currentPosition.getZ());
        }

        double distance = direction.getNorm();
        double ratio = stepSize / distance;

        Vector3D newPosition = currentPosition.add(direction.scalarMultiply(ratio));
        //currentPosition = newPosition;
        Point newPoint = new Point(newPosition.getX(),newPosition.getY(),newPosition.getZ());
        moveBee(newPoint, currentQuaternion, 667); // random number again
        System.out.println("Moved along axis " + axis + ": " + newPosition);
    }

    private double[] inspectCorners(List<Mat> corners) {

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

    private void moveCloserToArucoMarker(double[] aruco_middle, int current_target) {

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

        kinematics = api.getRobotKinematics();
        quaternion = kinematics.getOrientation();
        point = kinematics.getPosition();

        if (x_difference < -50) {
            new_point = new Point(point.getX(), point.getY() + 0.2, point.getZ());
            moveBee(new_point, quaternion, current_target); // move to right in y-axis
        }
        else if (x_difference > 50) {
            new_point = new Point(point.getX(), point.getY() - 0.2, point.getZ());
            moveBee(new_point, quaternion, current_target); // move to left in y-axis
        }

        if (x_difference < -30) {
            new_point = new Point(point.getX(), point.getY() + 0.1, point.getZ());
            moveBee(new_point, quaternion, current_target); // move to right in y-axis
        }
        else if (x_difference > 30) {
            new_point = new Point(point.getX(), point.getY() - 0.1, point.getZ());
            moveBee(new_point, quaternion, current_target); // move to left in y-axis
        }

        if (x_difference < -20) {
            new_point = new Point(point.getX(), point.getY() + 0.05, point.getZ());
            moveBee(new_point, quaternion, current_target); // move to right in y-axis
        }
        else if (x_difference > 20) {
            new_point = new Point(point.getX(), point.getY() - 0.05, point.getZ());
            moveBee(new_point, quaternion, current_target); // move to left in y-axis
        }



        kinematics = api.getRobotKinematics();
        quaternion = kinematics.getOrientation();
        point = kinematics.getPosition();

        if (y_difference <  -50) {
            new_point = new Point(point.getX() + 0.2, point.getY(), point.getZ());
            moveBee(new_point, quaternion, current_target); // move to down in x-axis
        }
        else if (y_difference > 50) {
            new_point = new Point(point.getX() - 0.2, point.getY(), point.getZ());
            moveBee(new_point, quaternion, current_target); // move to up in x-axis
        }

        if (y_difference <  -30) {
            new_point = new Point(point.getX() + 0.1, point.getY(), point.getZ());
            moveBee(new_point, quaternion, current_target); // move to down in x-axis
        }
        else if (y_difference > 30) {
            new_point = new Point(point.getX() - 0.1, point.getY(), point.getZ());
            moveBee(new_point, quaternion, current_target); // move to up in x-axis
        }

        if (y_difference <  -20) {
            new_point = new Point(point.getX() + 0.05, point.getY(), point.getZ());
            moveBee(new_point, quaternion, current_target); // move to down in x-axis
        }
        else if (y_difference > 20) {
            new_point = new Point(point.getX() - 0.05, point.getY(), point.getZ());
            moveBee(new_point, quaternion, current_target); // move to up in x-axis
        }
    }

    int imageProcessing_called = 0;
    private void imageProcessing(Dictionary dictionary, List<Mat> corners, DetectorParameters detectorParameters, Mat ids, int targetID) {

        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, "nearTarget" + targetID + "_" + imageProcessing_called + ".png");

        Mat colorImage = new Mat();
        // Convert the grayscale image to color
        Imgproc.cvtColor(grayImage, colorImage, Imgproc.COLOR_GRAY2BGR);

        Log.i(TAG, "TARGET " + targetID + " image processing");
        Aruco.detectMarkers(colorImage, dictionary, corners, ids, detectorParameters);
        Aruco.drawDetectedMarkers(colorImage, corners, ids, new Scalar( 0, 255, 0 ));

        Imgproc.putText(colorImage, "Aruco:"+ Arrays.toString(ids.get(0,0)), new org.opencv.core.Point(30.0, 80.0), 3, 0.5, new Scalar(255, 0, 0, 255), 1);

        api.saveMatImage(colorImage, "processedNearTarget" + current_target + "_" + imageProcessing_called+ ".png");
        imageProcessing_called++;

    }


}
