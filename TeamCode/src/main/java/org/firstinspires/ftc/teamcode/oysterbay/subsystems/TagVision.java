package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

/** Webcam + AprilTag processor, with the camera setup shared by every OpMode that looks for tags. */
public class TagVision {

    public static final String WEBCAM_NAME = "Webcam 1";

    // C270 works best at 640x480 in the SDK because intrinsics are provided at that res
    private static final Size RESOLUTION = new Size(640, 480);

    // Optional exposure/gain lock for stable detection in variable lighting
    private static final long EXPOSURE_MS = 12;
    private static final int GAIN_UNITS = 200;

    private static final int DASHBOARD_MAX_FPS = 30;

    private final AprilTagProcessor tagProc;
    private VisionPortal portal;

    /**
     * @param decimation processor decimation (1.0 = full detail, 2.0 ~ faster), or null for SDK default
     */
    public TagVision(HardwareMap hardwareMap, Float decimation) {
        tagProc = AprilTagProcessor.easyCreateWithDefaults();
        if (decimation != null) tagProc.setDecimation(decimation);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(RESOLUTION)
                .addProcessor(tagProc)
                .enableLiveView(true)
                .build();
    }

    /**
     * Blocks until the camera is streaming or {@code stopRequested} returns true.
     * Camera controls can only be touched after this.
     */
    public void waitForStreaming(BooleanSupplier stopRequested, Runnable idle) {
        while (!stopRequested.getAsBoolean() && !isStreaming()) {
            idle.run();
        }
    }

    public boolean isStreaming() {
        return portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Lock exposure and gain to fixed values (only works while streaming). */
    public void lockExposure() {
        if (!isStreaming()) return;

        ExposureControl exp = portal.getCameraControl(ExposureControl.class);
        if (exp != null) {
            exp.setMode(ExposureControl.Mode.Manual);
            exp.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (gain != null) {
            gain.setGain(Math.max(gain.getMinGain(), Math.min(GAIN_UNITS, gain.getMaxGain())));
        }
    }

    public void startDashboardStream() {
        FtcDashboard.getInstance().startCameraStream(portal, DASHBOARD_MAX_FPS);
    }

    public Object cameraState() {
        return portal == null ? "null" : portal.getCameraState();
    }

    /** Latest detections (never null). Each call copies the list, so fetch once per loop. */
    public List<AprilTagDetection> detections() {
        List<AprilTagDetection> dets = tagProc.getDetections();
        return dets == null ? Collections.<AprilTagDetection>emptyList() : dets;
    }

    /** Detection with the smallest forward range (z) that has a valid ftcPose, or null. */
    public static AprilTagDetection closest(List<AprilTagDetection> dets) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : dets) {
            if (d == null || d.ftcPose == null) continue;
            if (best == null || d.ftcPose.z < best.ftcPose.z) best = d;
        }
        return best;
    }

    public void close() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }
}
