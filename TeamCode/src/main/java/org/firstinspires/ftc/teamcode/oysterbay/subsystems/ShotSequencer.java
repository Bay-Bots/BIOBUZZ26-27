package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

/**
 * Trigger-driven shooting state machine shared by the driver TeleOp and the RPM tuner.
 *
 *  IDLE     -> light trigger pull spins the shooter up          -> SPINNING
 *  SPINNING -> full pull fires once the wheels are at speed
 *              (or the spin-up timeout passes)                  -> FIRING
 *  FIRING   -> tipper pulse finishes                            -> RECOVER
 *  RECOVER  -> trigger eased off: back to SPINNING if still held, else IDLE
 *
 * Each {@link Channel} is one trigger with its own target RPM (e.g. long and short shots).
 */
public class ShotSequencer {

    public enum State { IDLE, SPINNING, FIRING, RECOVER }

    /** A trigger input paired with the RPM it shoots at. */
    public static class Channel {
        public final String name;
        private final DoubleSupplier trigger;
        private final DoubleSupplier targetRpm;

        public Channel(String name, DoubleSupplier trigger, DoubleSupplier targetRpm) {
            this.name = name;
            this.trigger = trigger;
            this.targetRpm = targetRpm;
        }

        public double trigger()   { return trigger.getAsDouble(); }
        public double targetRpm() { return targetRpm.getAsDouble(); }
    }

    // Trigger thresholds
    public static final double SPIN_THRESHOLD  = 0.08;
    public static final double FIRE_THRESHOLD  = 0.92;
    public static final double RESET_THRESHOLD = 0.40;

    private static final double SPINUP_TIMEOUT_SEC = 1.5;
    private static final long FIRE_PULSE_MS = 220;

    private final Shooter shooter;
    private final Tipper tipper;
    private final Channel[] channels;

    private boolean requireAtSpeedToFire = true;
    private double atSpeedTolRpm = 150.0;

    private State state = State.IDLE;
    private Channel active = null;
    private boolean firedThisPress = false;

    private final ElapsedTime pulseTimer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();

    /** @param channels in priority order: the first one past SPIN_THRESHOLD wins */
    public ShotSequencer(Shooter shooter, Tipper tipper, Channel... channels) {
        this.shooter = shooter;
        this.tipper = tipper;
        this.channels = channels;
    }

    public void setRequireAtSpeedToFire(boolean require) { requireAtSpeedToFire = require; }
    public boolean isRequireAtSpeedToFire() { return requireAtSpeedToFire; }

    public void setAtSpeedTolRpm(double tolRpm) { atSpeedTolRpm = tolRpm; }
    public double getAtSpeedTolRpm() { return atSpeedTolRpm; }

    public State state() { return state; }

    /** Channel currently shooting, or null when idle. */
    public Channel active() { return active; }

    /** Target RPM of the active channel (0 when idle). */
    public double targetRpm() {
        return active == null ? 0.0 : active.targetRpm();
    }

    /** Advance the state machine one loop. */
    public void update() {
        Channel requested = requestedChannel();

        switch (state) {
            case IDLE:
                shooter.stop();
                tipper.rest();
                firedThisPress = false;
                active = null;

                if (requested != null) {
                    active = requested;
                    shooter.setRpm(active.targetRpm());
                    spinupTimer.reset();
                    state = State.SPINNING;
                }
                break;

            case SPINNING:
                if (requested == null) {
                    state = State.IDLE;
                    break;
                }

                // Switching triggers while spinning swaps targets smoothly
                active = requested;
                double target = active.targetRpm();
                shooter.setRpm(target);

                if (active.trigger() >= FIRE_THRESHOLD && !firedThisPress && readyToFire(target)) {
                    tipper.fire();
                    pulseTimer.reset();
                    firedThisPress = true;
                    state = State.FIRING;
                }
                break;

            case FIRING:
                shooter.setRpm(active.targetRpm());

                if (pulseTimer.milliseconds() >= FIRE_PULSE_MS) {
                    tipper.rest();
                    state = State.RECOVER;
                }
                break;

            case RECOVER:
                shooter.setRpm(active.targetRpm());

                if (active.trigger() < RESET_THRESHOLD) {
                    firedThisPress = false;

                    // Still lightly holding a trigger: go back to spinning
                    if (requested != null) {
                        active = requested;
                        spinupTimer.reset();
                        state = State.SPINNING;
                    } else {
                        state = State.IDLE;
                    }
                }
                break;
        }
    }

    private Channel requestedChannel() {
        for (Channel c : channels) {
            if (c.trigger() >= SPIN_THRESHOLD) return c;
        }
        return null;
    }

    private boolean readyToFire(double targetRpm) {
        return !requireAtSpeedToFire
                || shooter.isAtSpeed(targetRpm, atSpeedTolRpm)
                || spinupTimer.seconds() >= SPINUP_TIMEOUT_SEC;
    }
}
