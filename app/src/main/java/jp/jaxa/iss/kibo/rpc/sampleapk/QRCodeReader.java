package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.*;
import org.opencv.objdetect.QRCodeDetector;

public class QRCodeReader
{

    public String readQR(Mat img)
    {
        // assumes input is a grayscale image
        QRCodeDetector decoder = new QRCodeDetector();
        Mat points = new Mat();

        String data = decoder.detectAndDecodeCurved(img, points);

        if (!points.empty()) {
            return data;
        }
        else return "NO QR";
    }
}