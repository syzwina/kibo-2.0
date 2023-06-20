package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.*;
import org.opencv.objdetect.QRCodeDetector;

public class QRCodeReader
{

    public String readQR(Mat img)
    {
        // assumes input is a grayscale image
        QRCodeDetector decoder = new QRCodeDetector();
        ImageThresholding thresholder = new ImageThresholding();
        Mat points = new Mat();

        Mat thresholded = thresholder.applyThreshold(img);
        String data = decoder.detectAndDecode(thresholded, points);

        if (!points.empty()) {
            return data;
        }
        else return "NO QR";
    }
}