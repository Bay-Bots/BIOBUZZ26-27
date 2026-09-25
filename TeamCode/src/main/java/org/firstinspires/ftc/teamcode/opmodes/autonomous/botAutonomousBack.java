/*
Second autonomous version (Backwards start position):

- Robot stays STILL at the start
- Reads AprilTag first and locks the plan
- Immediately shoots
- Then drives "forward" for 1 second at the end

IMPORTANT:
This version assumes the robot is oriented BACKWARDS on the field.
So to move FORWARD at the end, we drive in the OPPOSITE direction compared to the reference program.
*/

// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomousBack.java
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "botAutonomous BackStart 2400", group = "OB")
public class botAutonomousBack extends OBAutoBase {

    private static final double SHOOTER_TARGET_RPM = 2300.0;

    private static final double STATIONARY_SCAN_SECONDS = 0.9;

    // End drive behavior (ONLY wheel use after shooting)
    private static final double END_DRIVE_SECONDS = 1.0;
    private static final double END_DRIVE_POWER = 0.30;

    @Override
    protected double shooterTargetRpm() {
        return SHOOTER_TARGET_RPM;
    }

    @Override
    protected void runAuto() {
        stopDrive();
        pointCameraLeft();

        Integer tag = scanForTag(STATIONARY_SCAN_SECONDS);
        runShootingPlan(tag);
        shooter.stop();

        // Robot faces backwards, so field-forward is robot-backward
        driveForwardFor(-END_DRIVE_POWER, END_DRIVE_SECONDS, "End drive (field forward)");
    }
}
