package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Four-motor mecanum drivetrain: motor setup, the cartesian mixer, and raw power helpers.
 *
 * Motor names expected in RC config:
 *  - "motorFrontRight", "motorFrontLeft", "motorBackRight", "motorBackLeft"
 */
public class MecanumDrivetrain {

    /** Which side's motors are reversed. TeleOp and the autos were tuned with opposite conventions. */
    public enum ReversedSide { LEFT, RIGHT }

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public MecanumDrivetrain(HardwareMap hardwareMap, ReversedSide reversedSide) {
        frontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        boolean leftReversed = reversedSide == ReversedSide.LEFT;
        setDirection(frontLeft,  leftReversed);
        setDirection(backLeft,   leftReversed);
        setDirection(frontRight, !leftReversed);
        setDirection(backRight,  !leftReversed);

        for (DcMotorEx m : new DcMotorEx[] { frontLeft, frontRight, backLeft, backRight }) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotorEx m : new DcMotorEx[] { frontLeft, frontRight, backLeft, backRight }) {
            m.setMode(mode);
        }
    }

    /**
     * Standard mecanum mixer. Inputs are nominally in [-1, 1]; wheel powers are scaled down
     * together so none exceeds 1.0 (preserves the direction of motion).
     */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, maxAbs(fl, fr, bl, br));
        setWheelPowers(fl / max, fr / max, bl / max, br / max);
    }

    /** Same power to all four wheels. */
    public void setAll(double power) {
        setWheelPowers(power, power, power, power);
    }

    public void stop() {
        setAll(0.0);
    }

    /** Direct per-wheel power, clipped to [-1, 1]. */
    public void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(Range.clip(fl, -1, 1));
        frontRight.setPower(Range.clip(fr, -1, 1));
        backLeft.setPower(Range.clip(bl, -1, 1));
        backRight.setPower(Range.clip(br, -1, 1));
    }

    private static void setDirection(DcMotorEx m, boolean reversed) {
        m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private static double maxAbs(double a, double b, double c, double d) {
        return Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
    }
}
