// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/RgbIndicatorLight.java
package org.firstinspires.ftc.teamcode.opmodes;

/*
 * Colors for the goBILDA RGB Indicator Light (PWM controlled).
 *
 * The light plugs into a servo port and is set up in the Robot Configuration as a Servo. The
 * servo position sent to it picks the color: 0.0 is off, 0.277 to 0.722 runs through the
 * spectrum from red to violet, and 1.0 is white. These are goBILDA's positions for the SDK's
 * default servo range (600-2400 us). Positions in between the ones below blend the colors.
 *
 * Used by the Limelight Tag Lock programs, e.g.  light.setPosition(RgbIndicatorLight.GREEN);
 */
public final class RgbIndicatorLight {

    public static final double OFF    = 0.0;
    public static final double RED    = 0.277;
    public static final double ORANGE = 0.333;
    public static final double YELLOW = 0.388;
    public static final double SAGE   = 0.444;
    public static final double GREEN  = 0.500;
    public static final double AZURE  = 0.555;
    public static final double BLUE   = 0.611;
    public static final double INDIGO = 0.666;
    public static final double VIOLET = 0.722;
    public static final double WHITE  = 1.0;

    private RgbIndicatorLight() {}

    /** Name of a color above, for the Driver Station. */
    public static String name(double color) {
        if (color == OFF)    return "Off";
        if (color == RED)    return "Red";
        if (color == ORANGE) return "Orange";
        if (color == YELLOW) return "Yellow";
        if (color == SAGE)   return "Sage";
        if (color == GREEN)  return "Green";
        if (color == AZURE)  return "Azure";
        if (color == BLUE)   return "Blue";
        if (color == INDIGO) return "Indigo";
        if (color == VIOLET) return "Violet";
        if (color == WHITE)  return "White";
        return String.format("position %.3f", color);
    }
}
