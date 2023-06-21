package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.*;
import org.opencv.objdetect.QRCodeDetector;
import org.opencv.imgproc.Imgproc;

public class QRCodeReader
{

    public String readQR(Mat img)
    {

        QRCodeDetector decoder = new QRCodeDetector();
        ImageThresholding thresholder = new ImageThresholding();
        Mat points = new Mat();

        Mat thresholded = thresholder.applyThreshold(img);

        // Convert binary image to grayscale
        Mat grayscale = new Mat();
        Imgproc.cvtColor(thresholded, grayscale, Imgproc.COLOR_GRAY2BGR);

        String data = decoder.detectAndDecode(grayscale, points);

        if (!points.empty()) {
            return data;
        }
        else return "NO QR";
    }
}