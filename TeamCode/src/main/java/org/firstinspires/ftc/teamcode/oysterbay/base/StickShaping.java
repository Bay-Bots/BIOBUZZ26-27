package org.firstinspires.ftc.teamcode.oysterbay.base;

/** Joystick input shaping helpers. */
public final class StickShaping {

    private StickShaping() {}

    /** Zero out values inside the deadzone. */
    public static double deadband(double v, double d) {
        return (Math.abs(v) < d) ? 0.0 : v;
    }

    /** expo = sign(v)*|v|^p; p > 1 softens the low end, p = 1 is linear. */
    public static double expo(double v, double p) {
        return Math.signum(v) * Math.pow(Math.abs(v), p);
    }

    /** Deadband, then optional expo. */
    public static double shape(double v, double deadband, boolean applyExpo, double expo) {
        v = deadband(v, deadband);
        return applyExpo ? expo(v, expo) : v;
    }
}
