package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Intake roller. Negative power pulls balls in / feeds the shooter; positive clears them out. */
public class Intake {

    public static final double FEED_POWER    = -1.0;
    public static final double REVERSE_POWER = +1.0;

    private final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // If intake runs backwards, uncomment:
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) { motor.setPower(power); }
    public void feed()                 { setPower(FEED_POWER); }
    public void reverse()              { setPower(REVERSE_POWER); }
    public void stop()                 { setPower(0.0); }

    /** Feed while held, otherwise stop. */
    public void feedWhile(boolean held) {
        if (held) feed(); else stop();
    }
}
