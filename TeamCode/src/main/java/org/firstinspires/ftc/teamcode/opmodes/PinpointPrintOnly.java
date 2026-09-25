// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/PinpointPrintOnly.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Odometry;

/** Prints the Pinpoint pose while you push the robot by hand. */
@TeleOp(name = "Pinpoint Print ONLY", group = "Test")
public class PinpointPrintOnly extends LinearOpMode {

    // Set true to apply pod offsets/directions (as the autos do) and zero at start
    private static final boolean APPLY_GEOMETRY_AND_RESET = false;

    private static final double MM_PER_INCH = 25.4;

    @Override
    public void runOpMode() {
        // Pod resolution is set by the constructor
        Odometry odometry = new Odometry(hardwareMap);
        if (APPLY_GEOMETRY_AND_RESET) {
            odometry.applyRobotGeometry();
            odometry.resetPosAndIMU();
        }

        telemetry.addLine("Pinpoint Print ONLY ready.");
        telemetry.addLine("Press START, then move robot by hand.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Pose2D pose = odometry.update();

            telemetry.addData("Status", odometry.status());
            if (pose != null) {
                odometry.addPoseTelemetry(telemetry, true);
                telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.MM) / MM_PER_INCH);
                telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.MM) / MM_PER_INCH);
            } else {
                telemetry.addLine("Pose is null (Pinpoint not returning pose yet).");
            }

            telemetry.update();
            idle();
        }
    }
}
