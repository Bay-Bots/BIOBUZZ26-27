package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oysterbay.subsystems.TagVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/** AprilTag test: shows every visible tag ID and the pose of the closest one. */
@Autonomous(name = "AT_Quickstart", group = "OB")
public class AT_Quickstart extends LinearOpMode {

    // Optional: lock exposure/gain for stable detection in variable lighting
    private static final boolean LOCK_CAMERA_EXPOSURE = false;

    // Optional: decimation (trade detail for speed). 1.0 = full detail, 2.0 ~ faster.
    private static final Float DECIMATION = 2.0f;

    private TagVision vision;

    @Override
    public void runOpMode() {
        vision = new TagVision(hardwareMap, DECIMATION);
        vision.waitForStreaming(this::isStopRequested, () -> { sleep(10); idle(); });
        if (LOCK_CAMERA_EXPOSURE) vision.lockExposure();

        // Dashboard streaming + merged telemetry (RC + Dashboard)
        vision.startDashboardStream();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("AprilTag init complete. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            showDetections(vision.detections());
            telemetry.update();
            idle();
        }

        vision.close();
    }

    private void showDetections(List<AprilTagDetection> dets) {
        if (dets.isEmpty()) {
            telemetry.addLine("No tags.");
            return;
        }

        telemetry.addData("Detections", dets.size());
        telemetry.addData("IDs", idList(dets));

        AprilTagDetection best = TagVision.closest(dets);
        if (best == null) {
            telemetry.addLine("Detections present, but no reliable ftcPose yet.");
            return;
        }

        telemetry.addLine("--- Best ---");
        telemetry.addData("ID", best.id);
        telemetry.addData("Z (forward, m)", "%.3f", best.ftcPose.z);
        telemetry.addData("X (left+, m)", "%.3f", best.ftcPose.x);
        telemetry.addData("Y (up+, m)", "%.3f", best.ftcPose.y);
        telemetry.addData("Yaw (deg)",   "%.1f", best.ftcPose.yaw);
        telemetry.addData("Pitch (deg)", "%.1f", best.ftcPose.pitch);
        telemetry.addData("Roll (deg)",  "%.1f", best.ftcPose.roll);
    }

    private static String idList(List<AprilTagDetection> dets) {
        StringBuilder ids = new StringBuilder();
        for (AprilTagDetection d : dets) {
            ids.append(d.id).append(" ");
        }
        return ids.toString().trim();
    }
}
