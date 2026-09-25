// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/oysterbay/base/RobotStructure.java
package org.firstinspires.ftc.teamcode.oysterbay.base;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.oysterbay.subsystems.MecanumDrivetrain;

/**
 * TeleOp-only drivetrain structure to be reused across programs.
 * Owns the drivetrain and the hub bulk-read cache, and maps a gamepad onto the drive.
 */
public class RobotStructure {

    private MecanumDrivetrain drive;
    private BulkReads bulkReads;

    // --- Tunables ---
    private static final double DEADBAND = 0.05;  // stick deadzone
    private static final double EXPO_DRIVE = 2.0; // >1 softens low-end; 1.0 = linear

    // +20% turning speed
    private static final double TURN_BOOST = 1.20;

    public void init(HardwareMap hardwareMap) {
        // Call clearBulkCache() at the top of every loop()
        bulkReads = BulkReads.manual(hardwareMap);

        // Right side reversed: +power = forward for all wheels
        drive = new MecanumDrivetrain(hardwareMap, MecanumDrivetrain.ReversedSide.RIGHT);
    }

    /** Invalidate the bulk-read cache. Must be called once at the start of every loop(). */
    public void clearBulkCache() {
        bulkReads.clear();
    }

    /**
     * Drive from a gamepad (left stick = translation, right stick X = turn).
     *
     * @param gp the driver gamepad
     * @param squaredInputs true to apply exponential shaping (smoother low speed)
     * @param speedMult overall speed scale (e.g., 1.0 normal, 0.4 slow)
     */
    public void driveFromGamepad(Gamepad gp, boolean squaredInputs, double speedMult) {
        // Front/back flipped; strafe and rotate inverted to match
        double forward = StickShaping.shape(-gp.left_stick_y,  DEADBAND, squaredInputs, EXPO_DRIVE);
        double strafe  = StickShaping.shape(-gp.left_stick_x,  DEADBAND, squaredInputs, EXPO_DRIVE);
        double turn    = StickShaping.shape(-gp.right_stick_x, DEADBAND, squaredInputs, EXPO_DRIVE) * TURN_BOOST;

        drive.drive(forward * speedMult, strafe * speedMult, turn * speedMult);
    }

    /** Direct power set by wheel (clipped to [-1, 1]). */
    public void setDriverMotorPower(double frontRight, double frontLeft,
                                    double backRight, double backLeft) {
        drive.setWheelPowers(frontLeft, frontRight, backLeft, backRight);
    }

    /** Stop all drive motors. */
    public void setDriverPowerZERO() {
        drive.stop();
    }

    /** Simple strafes (normalized). */
    public void translateRight(double m) { drive.drive(0, m, 0); }
    public void translateLeft (double m) { drive.drive(0, -m, 0); }
}
