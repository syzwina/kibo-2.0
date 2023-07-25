package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.core.Scalar;
import org.opencv.aruco.DetectorParameters;

import java.util.Arrays;
import java.util.List;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * This class provides image processing functionalities for Near Target Identification.
 * It inherits from KiboRpcService to use the provided API for communication with the Kibo Robot Programming Challenge.
 */
public class ImageProcessing extends KiboRpcService {

    /**
     * Tag used for logging purposes. It helps identify log messages from this class.
     */
    private final String TAG = this.getClass().getSimpleName();

    /**
     * Counter to keep track of how many times the imageProcessing method has been called.
     */
    private int imageProcessing_called = 0;

    /**
     * Processes the provided image using ArUco marker detection and labeling.
     *
     * @param dictionary        The ArUco dictionary for marker detection.
     * @param corners           List to store the detected marker corners.
     * @param detectorParameters Parameters for the marker detector.
     * @param ids               Matrix to store the detected marker IDs.
     * @param targetID          The target ID for which the image is being processed.
     */
    private void imageProcessing(Dictionary dictionary, List<Mat> corners, DetectorParameters detectorParameters, Mat ids, int targetID) {

        Mat grayImage = api.getMatNavCam();
        api.saveMatImage(grayImage, "nearTarget" + targetID + "_" + imageProcessing_called + ".png");
        Log.i(TAG+"/imageProcessing", "Image has been saved in Black and White");

        Mat colorImage = new Mat();
        // Convert the grayscale image to color
        Imgproc.cvtColor(grayImage, colorImage, Imgproc.COLOR_GRAY2BGR);

        Log.i(TAG+"/imageProcessing", "TARGET " + targetID + " image processing");
        Aruco.detectMarkers(colorImage, dictionary, corners, ids, detectorParameters);
        Aruco.drawDetectedMarkers(colorImage, corners, ids, new Scalar( 0, 255, 0 ));

        Imgproc.putText(colorImage, "Aruco:"+ Arrays.toString(ids.get(0,0)), new org.opencv.core.Point(30.0, 80.0), 3, 0.5, new Scalar(255, 0, 0, 255), 1);
        Log.i(TAG+"imageProcessing", "Aruco marker has been labeled");

        api.saveMatImage(colorImage, "processedNearTarget" + targetID + "_" + imageProcessing_called+ ".png");
        Log.i(TAG+"imageProcessing", "Image has been saved in Colour");
        imageProcessing_called++;

    }

}