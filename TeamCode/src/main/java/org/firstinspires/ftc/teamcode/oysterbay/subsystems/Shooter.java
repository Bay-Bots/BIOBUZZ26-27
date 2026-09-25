package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Two-wheel flywheel shooter under velocity control (setVelocity).
 *
 * Both motors are Direction.FORWARD; the right motor is commanded with a negative velocity so the
 * wheels spin inward. All RPM values in this class are "shooting-positive" regardless of wiring.
 */
public class Shooter {

    /** goBILDA Yellow Jacket encoder at the motor shaft. Verify for your exact motor. */
    public static final double TICKS_PER_REV = 28.0;

    private static final int LEFT_CMD_SIGN  = +1;
    private static final int RIGHT_CMD_SIGN = -1;

    // Trim multipliers: tune if one side consistently runs high/low
    private static final double LEFT_TRIM  = 1.00;
    private static final double RIGHT_TRIM = 1.00;

    private final DcMotorEx left;
    private final DcMotorEx right;

    // Last commanded RPM; setVelocity() is only sent to the hub when this changes
    private double lastRpm = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        left  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        right = hardwareMap.get(DcMotorEx.class, "shooterRight");

        for (DcMotorEx m : new DcMotorEx[] { left, right }) {
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        stop();
    }

    /** Command both wheels to a shooting-positive RPM (negative = run backwards). */
    public void setRpm(double rpm) {
        if (rpm == lastRpm) return;
        lastRpm = rpm;

        double tps = rpmToTicksPerSec(rpm);
        left.setVelocity(LEFT_CMD_SIGN * tps * LEFT_TRIM);
        right.setVelocity(RIGHT_CMD_SIGN * tps * RIGHT_TRIM);
    }

    public void stop() {
        setRpm(0.0);
    }

    public double leftRpm()  { return ticksPerSecToRpm(left.getVelocity()  * LEFT_CMD_SIGN); }
    public double rightRpm() { return ticksPerSecToRpm(right.getVelocity() * RIGHT_CMD_SIGN); }

    /** Raw encoder values, for wiring/sign debugging. */
    public int leftPosition()        { return left.getCurrentPosition(); }
    public int rightPosition()       { return right.getCurrentPosition(); }
    public double leftRawVelocity()  { return left.getVelocity(); }
    public double rightRawVelocity() { return right.getVelocity(); }

    /** Reads both wheels and checks them against the target. */
    public boolean isAtSpeed(double targetRpm, double tolRpm) {
        return bothWithin(leftRpm(), rightRpm(), targetRpm, tolRpm);
    }

    /** At-speed check on readings the caller already has (avoids re-reading the motors). */
    public static boolean bothWithin(double leftRpm, double rightRpm, double targetRpm, double tolRpm) {
        double tgt = Math.abs(targetRpm);
        return Math.abs(leftRpm - tgt) <= tolRpm && Math.abs(rightRpm - tgt) <= tolRpm;
    }

    public static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    public static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }
}
