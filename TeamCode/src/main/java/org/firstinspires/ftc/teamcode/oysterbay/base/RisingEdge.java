package org.firstinspires.ftc.teamcode.oysterbay.base;

/** Detects the moment a button goes from released to pressed. One instance per button. */
public class RisingEdge {

    private boolean previous = false;

    /** @return true only on the loop where {@code pressed} first becomes true */
    public boolean update(boolean pressed) {
        boolean edge = pressed && !previous;
        previous = pressed;
        return edge;
    }
}
