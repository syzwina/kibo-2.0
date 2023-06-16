package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.HashMap;

public class QRCodeMapper {
    private HashMap<String, String> qrCodeMap;

    public QRCodeMapper() {
        qrCodeMap = new HashMap<>();
        // Add key-value pairs to the map
        qrCodeMap.put("JEM", "STAY_AT_JEM");
        qrCodeMap.put("COLUMBUS", "GO_TO_COLUMBUS");
        qrCodeMap.put("RACK1", "CHECK_RACK_1");
        qrCodeMap.put("ASTROBEE", "I_AM_HERE");
        qrCodeMap.put("INTBALL", "LOOKING_FORWARD_TO_SEE_YOU");
        qrCodeMap.put("BLANK", "NO_PROBLEM");
    }

    public String getValue(String key) {
        if (qrCodeMap.containsKey(key)) {
            return qrCodeMap.get(key);
        } else {
            return "Key not found";
        }
    }
}


