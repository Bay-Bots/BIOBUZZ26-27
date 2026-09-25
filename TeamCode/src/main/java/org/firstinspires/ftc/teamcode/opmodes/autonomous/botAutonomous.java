/*
Two things you may need to flip after the first test:

1. If the robot strafes away from the Y target, change
PINPOINT_Y_INCREASES_WHEN_LEFT (OBAutoBase) to the opposite value.

2. If the pickup motor spits out instead of intaking, reverse
the intake direction (the commented line in Intake).
 */
// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomous.java
// 1820 Auton Regular
// 2400 Back corner
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Regular start: drive a little, stop and read the AprilTag, finish the drive, shoot in the
 * order the tag calls for, then strafe off.
 */
@Autonomous(name = "botAutonomous", group = "OB")
public class botAutonomous extends OBAutoBase {

    private static final double SHOOTER_TARGET_RPM = 1800.0;

    // Drive + scan behavior
    private static final double DRIVE_TOTAL_SECONDS = 2.5;
    private static final double PREMOVE_SECONDS = 1.0;
    private static final double STATIONARY_SCAN_SECONDS = 0.9;
    private static final double SCAN_DRIVE_POWER = 0.30;
    private static final long POST_DECISION_PAUSE_MS = 100;

    // Final strafe off the line
    private static final double END_STRAFE_POWER = 0.35;
    private static final long END_STRAFE_MS = 700;

    @Override
    protected double shooterTargetRpm() {
        return SHOOTER_TARGET_RPM;
    }

    @Override
    protected void runAuto() {
        pointCameraLeft();

        // Move a little, then decide the tag while STOPPED
        driveForwardFor(SCAN_DRIVE_POWER, PREMOVE_SECONDS, "Drive (pre-scan)");
        Integer tag = scanForTag(STATIONARY_SCAN_SECONDS);

        // Finish the drive so total distance matches the original 2.5 s drive
        driveForwardFor(SCAN_DRIVE_POWER, Math.max(0.0, DRIVE_TOTAL_SECONDS - PREMOVE_SECONDS), "Drive (post-scan)");
        sleep(POST_DECISION_PAUSE_MS);

        runShootingPlan(tag);
        driveFor(0.0, END_STRAFE_POWER, 0.0, END_STRAFE_MS);
    }
}
