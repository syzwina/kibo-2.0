package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class ImageThresholding {
    public Mat applyThreshold(Mat image) {
        Mat grayImage = new Mat();
        Mat thresholdedImage = new Mat();

        // Convert the image to grayscale
        Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);

        // Apply thresholding
        Imgproc.threshold(grayImage, thresholdedImage, 0, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        return thresholdedImage;
    }
}