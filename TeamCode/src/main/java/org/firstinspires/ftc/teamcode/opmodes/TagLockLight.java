// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/TagLockLight.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * goBILDA RGB Indicator Light that shows what the Limelight tag lock is doing.
 * This is not an OpMode. The tag-lock OpModes (LimelightTagLock, LimelightTagLockRpmStepUp,
 * LimelightTagLockAutoRpm, ShooterDistanceTuner) use it, so the light means the same thing
 * in all of them:
 *
 *   SEARCHING    Section 1: the camera is sweeping, looking for an AprilTag    red, flashing
 *   TAG IN VIEW  Section 2: a tag is in view, the camera is turning to center  yellow, flashing fast
 *                it (the tag is still off to the side of the camera's view)
 *   LOCKED       Section 3: the camera is centered on the tag                  green, solid
 *   NO CAMERA    the Limelight is not sending data (unplugged, still booting)  violet, flashing slowly
 *
 * Change the colors and flash speeds in the "Colors" section below.
 *
 * Robot Configuration:
 *   - goBILDA RGB Indicator Light on a servo port, type Servo      name: "rgbLight"
 *   If the Robot Configuration has no "rgbLight", the light is skipped and the OpModes still run.
 */
public final class TagLockLight {

    public static final String LIGHT_NAME = "rgbLight";

    // =========================
    // goBILDA RGB Indicator Light colors (servo positions, SDK default 600-2400 us range)
    // =========================
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

    // =========================
    // Colors: what each state shows. FLASHES_PER_SEC = 0 means solid (no flashing).
    // =========================
    private static final double SEARCHING_COLOR           = RED;
    private static final double SEARCHING_FLASHES_PER_SEC = 2.0;

    private static final double TAG_IN_VIEW_COLOR           = YELLOW;
    private static final double TAG_IN_VIEW_FLASHES_PER_SEC = 5.0;

    private static final double LOCKED_COLOR           = GREEN;
    private static final double LOCKED_FLASHES_PER_SEC = 0.0;

    private static final double NO_CAMERA_COLOR           = VIOLET;
    private static final double NO_CAMERA_FLASHES_PER_SEC = 1.0;

    public enum State { OFF, NO_CAMERA, SEARCHING, TAG_IN_VIEW, LOCKED }

    private final Servo light;                     // null if "rgbLight" is not in the Robot Configuration
    private final ElapsedTime sinceStateChange = new ElapsedTime();
    private State state = State.OFF;
    private double lastPosition = -1.0;            // last position sent, so it is only sent when it changes

    public TagLockLight(HardwareMap hardwareMap) {
        light = hardwareMap.tryGet(Servo.class, LIGHT_NAME);
        off();
    }

    /** True if the Robot Configuration has the light. */
    public boolean isConnected() {
        return light != null;
    }

    /**
     * Shows a state. Call it every loop so the flashing keeps going. A new state starts with
     * the light ON, so a change shows up right away.
     */
    public void show(State newState) {
        if (newState != state) {
            state = newState;
            sinceStateChange.reset();
        }

        double color = colorOf(state);
        double flashesPerSec = flashesPerSecOf(state);
        boolean lit = true;
        if (flashesPerSec > 0) {
            // On for the first half of each flash, off for the second half
            double cycle = sinceStateChange.seconds() * flashesPerSec;
            lit = (cycle - Math.floor(cycle)) < 0.5;
        }
        setPosition(lit ? color : OFF);
    }

    /** Turns the light off (use when the OpMode stops). */
    public void off() {
        state = State.OFF;
        setPosition(OFF);
    }

    /** For the Driver Station, for example "LOCKED: green, solid". */
    public String describe() {
        if (light == null) return "not in the Robot Configuration (add a Servo named \"" + LIGHT_NAME + "\")";
        if (state == State.OFF) return "off";

        double flashesPerSec = flashesPerSecOf(state);
        return String.format("%s: %s, %s", stateName(state), colorName(colorOf(state)),
                flashesPerSec > 0 ? String.format("flashing %.0f/sec", flashesPerSec) : "solid");
    }

    private void setPosition(double position) {
        if (light == null || position == lastPosition) return;
        light.setPosition(position);
        lastPosition = position;
    }

    private static double colorOf(State s) {
        switch (s) {
            case NO_CAMERA:   return NO_CAMERA_COLOR;
            case SEARCHING:   return SEARCHING_COLOR;
            case TAG_IN_VIEW: return TAG_IN_VIEW_COLOR;
            case LOCKED:      return LOCKED_COLOR;
            default:          return OFF;
        }
    }

    private static double flashesPerSecOf(State s) {
        switch (s) {
            case NO_CAMERA:   return NO_CAMERA_FLASHES_PER_SEC;
            case SEARCHING:   return SEARCHING_FLASHES_PER_SEC;
            case TAG_IN_VIEW: return TAG_IN_VIEW_FLASHES_PER_SEC;
            case LOCKED:      return LOCKED_FLASHES_PER_SEC;
            default:          return 0.0;
        }
    }

    private static String stateName(State s) {
        switch (s) {
            case NO_CAMERA:   return "NO CAMERA DATA";
            case SEARCHING:   return "SEARCHING";
            case TAG_IN_VIEW: return "TAG IN VIEW";
            case LOCKED:      return "LOCKED";
            default:          return "OFF";
        }
    }

    private static String colorName(double position) {
        double[] positions = { OFF, RED, ORANGE, YELLOW, SAGE, GREEN, AZURE, BLUE, INDIGO, VIOLET, WHITE };
        String[] names = { "off", "red", "orange", "yellow", "sage", "green", "azure", "blue", "indigo", "violet", "white" };
        int best = 0;
        for (int i = 1; i < positions.length; i++) {
            if (Math.abs(positions[i] - position) < Math.abs(positions[best] - position)) best = i;
        }
        return names[best];
    }
}
