package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Left/right ball holder servos. TeleOp and auto were tuned with different left positions,
 * so each has its own factory.
 */
public class Holders {

    private static final double RIGHT_OPEN   = 0.00;
    private static final double RIGHT_CLOSED = 0.40;

    private final Servo left;
    private final Servo right;
    private final double leftOpen;
    private final double leftClosed;

    private Holders(HardwareMap hardwareMap, double leftOpen, double leftClosed) {
        this.left  = hardwareMap.get(Servo.class, "leftHolderServo");
        this.right = hardwareMap.get(Servo.class, "rightHolderServo");
        this.leftOpen = leftOpen;
        this.leftClosed = leftClosed;
        closeBoth();
    }

    public static Holders forTeleOp(HardwareMap hardwareMap) {
        return new Holders(hardwareMap, 0.50, 0.15);
    }

    public static Holders forAuto(HardwareMap hardwareMap) {
        return new Holders(hardwareMap, 0.53, 0.10);
    }

    public void openLeft()   { left.setPosition(leftOpen); }
    public void closeLeft()  { left.setPosition(leftClosed); }
    public void openRight()  { right.setPosition(RIGHT_OPEN); }
    public void closeRight() { right.setPosition(RIGHT_CLOSED); }

    public void openBoth()  { openLeft();  openRight(); }
    public void closeBoth() { closeLeft(); closeRight(); }

    /** D-pad: left/right open that side, up opens both, down closes both. */
    public void updateFromDpad(Gamepad gp) {
        if (gp.dpad_left)  openLeft();
        if (gp.dpad_right) openRight();
        if (gp.dpad_up)    openBoth();
        if (gp.dpad_down)  closeBoth();
    }
}
