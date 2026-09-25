package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** goBILDA Pinpoint odometry computer with this robot's pod geometry. */
public class Odometry {

    public static final String DEVICE_NAME = "pinpoint";

    // xOffset: sideways offset of the X (forward) pod, right is negative
    // yOffset: forward offset of the Y (strafe) pod, forward is positive
    private static final double X_OFFSET_MM = -157.5;
    private static final double Y_OFFSET_MM = 15.0;

    // X counts should increase moving forward; Y counts should increase moving left
    private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D pose;

    public Odometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    /** Apply pod offsets and encoder directions (resolution is always set in the constructor). */
    public void applyRobotGeometry() {
        pinpoint.setOffsets(X_OFFSET_MM, Y_OFFSET_MM);
        pinpoint.setEncoderDirections(X_ENCODER_DIR, Y_ENCODER_DIR);
    }

    /** Zero the pose and recalibrate the IMU. The robot must be stationary for ~350 ms afterwards. */
    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    /** Pull fresh data from the device; returns the new pose (may be null before the first read). */
    public Pose2D update() {
        pinpoint.update();
        pose = pinpoint.getPosition();
        return pose;
    }

    /** Pose from the most recent update(). */
    public Pose2D pose() { return pose; }

    public String status() {
        return String.valueOf(pinpoint.getDeviceStatus());
    }

    /** Adds X/Y (mm) and optionally heading of the last pose to telemetry; no-op if pose is null. */
    public void addPoseTelemetry(Telemetry telemetry, boolean includeHeading) {
        if (pose == null) return;
        telemetry.addData("Odo X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
        telemetry.addData("Odo Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
        if (includeHeading) {
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        }
    }
}
