package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Pair of continuous-rotation "trap" servos that push balls toward the shooter. */
public class Trap {

    private static final double FEED_POWER    = 0.6;
    private static final double REVERSE_POWER = 0.2;

    private final CRServo left;
    private final CRServo right;

    public Trap(HardwareMap hardwareMap) {
        left  = hardwareMap.get(CRServo.class, "servoTrapLeft");
        right = hardwareMap.get(CRServo.class, "servoTrapRight");
    }

    /** Raw power per side (sides spin in opposite directions to move balls the same way). */
    public void set(double leftPower, double rightPower) {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public void feed()    { set(-FEED_POWER, FEED_POWER); }
    public void reverse() { set(REVERSE_POWER, -REVERSE_POWER); }
    public void stop()    { set(0.0, 0.0); }

    /** Feed while held, otherwise stop. */
    public void feedWhile(boolean held) {
        if (held) feed(); else stop();
    }
}
