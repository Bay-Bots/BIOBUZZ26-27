package org.firstinspires.ftc.teamcode.oysterbay.base;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Lynx bulk caching: all encoder/velocity values on a hub arrive in one transaction.
 *
 *  - MANUAL (iterative OpModes): call {@link #clear()} at the top of every loop().
 *  - AUTO (LinearOpModes): refreshes automatically whenever a value is read a second time.
 */
public class BulkReads {

    private final List<LynxModule> hubs;

    private BulkReads(HardwareMap hardwareMap, LynxModule.BulkCachingMode mode) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(mode);
        }
    }

    public static BulkReads manual(HardwareMap hardwareMap) {
        return new BulkReads(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
    }

    public static BulkReads auto(HardwareMap hardwareMap) {
        return new BulkReads(hardwareMap, LynxModule.BulkCachingMode.AUTO);
    }

    /** Invalidate the cache so the next read does a fresh bulk read. */
    public void clear() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
