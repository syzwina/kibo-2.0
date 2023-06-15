package jp.jaxa.iss.kibo.rpc.sampleapk;


import org.opencv.core.*;
import org.opencv.objdetect.QRCodeDetector;

public class QRCodeReader
{
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public String readQR(Mat img)
    {

        QRCodeDetector decoder = new QRCodeDetector();
        Mat points = new Mat();
        String data = decoder.detectAndDecode(img, points);

        if (!points.empty()) {
            return data;
        }
        else return "NO QR";
    }
}