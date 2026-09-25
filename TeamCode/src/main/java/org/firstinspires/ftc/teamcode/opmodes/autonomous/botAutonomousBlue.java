/*
Blue backwards-start autonomous:

- Robot stays STILL at the start and reads the AprilTag
- Drives backwards to mid-field and turns
- Shoots in the order the tag calls for

IMPORTANT:
This version assumes the robot is oriented BACKWARDS on the field.
*/

// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomousBlue.java
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "botAutonomous BackStart 2400 - blue", group = "OB")
public class botAutonomousBlue extends OBAutoBase {

    private static final double SHOOTER_TARGET_RPM = 2000.0;

    private static final double STATIONARY_SCAN_SECONDS = 0.9;

    // Drive to mid-field, then turn toward the goal
    private static final double TO_MID_POWER = 0.30;
    private static final double TO_MID_SECONDS = 4.0;
    private static final double TURN_POWER = -0.3;
    private static final long TURN_MS = 700;

    @Override
    protected double shooterTargetRpm() {
        return SHOOTER_TARGET_RPM;
    }

    @Override
    protected void runAuto() {
        stopDrive();
        pointCameraLeft();

        Integer tag = scanForTag(STATIONARY_SCAN_SECONDS);

        // Robot faces backwards, so field-forward is robot-backward
        driveForwardFor(-TO_MID_POWER, TO_MID_SECONDS, "Drive to mid");
        driveFor(0.0, 0.0, TURN_POWER, TURN_MS);

        runShootingPlan(tag);
    }
}
