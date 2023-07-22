package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class ImageThresholding {
    public Mat applyThreshold(Mat image) {
        Mat thresholdedImage = new Mat();

        // Apply thresholding
        Imgproc.threshold(image, thresholdedImage, 0, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        return thresholdedImage;
    }
}