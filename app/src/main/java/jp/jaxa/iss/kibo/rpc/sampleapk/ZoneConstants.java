package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.Arrays;
import java.util.List;

public final class ZoneConstants {

    // initializing zones
    static final KeepOutZone KOZ01 = new KeepOutZone(10.783f, -9.8899f, 4.8385f, 11.071f, -9.6929f, 5.0665f);
    static final KeepOutZone KOZ02 = new KeepOutZone(10.8652f, -9.0734f, 4.3861f, 10.9628f, -8.7314f, 4.6401f);
    static final KeepOutZone KOZ03 = new KeepOutZone(10.185f, -8.3826f, 4.1475f, 11.665f, -8.2826f, 4.6725f);
    static final KeepOutZone KOZ04 = new KeepOutZone(10.7955f, -8.0635f, 5.1055f, 11.3525f, -7.7305f, 5.1305f);
    static final KeepOutZone KOZ05 = new KeepOutZone(10.563f, -7.1449f, 4.6544f, 10.709f, -6.8099f, 4.8164f);
    static final KeepOutZone CURRENT_KOZ = new KeepOutZone(0f,0f,0f,0f,0f,0f); // not really needed

    static final List<KeepOutZone> ZONES = Arrays.asList(KOZ01,KOZ02,KOZ03,KOZ04,KOZ05);

    static final KeepInZone KIZ01 = new KeepInZone(10.3f, -10.2f, 4.32f, 11.55f, -6.0f, 5.57f);
    static final KeepInZone KIZ02 = new KeepInZone(9.5f, -10.5f, 4.02f, 10.5f, -9.6f, 4.8f);
    
    private ZoneConstants() {
        
    }
}