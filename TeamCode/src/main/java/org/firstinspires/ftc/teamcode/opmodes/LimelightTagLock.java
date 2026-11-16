// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/LimelightTagLock.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Limelight 3A on a servo: find AprilTag 35 or 36, center the camera on it, and stay locked on.
 *
 * The Limelight is mounted on a servo that turns it left and right. The program runs in three
 * sections, and the Driver Station shows which one is running:
 *
 *   SECTION 1 - SEARCH   The servo slowly sweeps the camera back and forth until the Limelight
 *                        sees AprilTag 35 or 36.
 *   SECTION 2 - ALIGN    The servo keeps adjusting its position until tx is zero degrees.
 *                        (tx is the Limelight's horizontal angle from the center of its view
 *                        to the tag. It is positive when the tag is to the right.)
 *   SECTION 3 - LOCKED   The target is locked. If the tag or the robot moves, the servo turns
 *                        the camera to keep tx at zero.
 *
 * If the tag stays out of sight for LOST_TIMEOUT_SEC in Section 2 or 3, the program goes back
 * to Section 1 and sweeps toward the side where the tag was last seen.
 *
 * Robot Configuration:
 *   - Limelight 3A (plugged into USB, listed with the webcams)        name: "limelight"
 *   - Standard position servo (not continuous rotation) that turns
 *     the Limelight, on a servo port, type Servo                      name: "camServo"
 *
 * Limelight pipeline (set up in the Limelight web interface):
 *   - PIPELINE_INDEX must be an AprilTag pipeline (family 36h11)
 *
 * Before the first run, check the "Camera servo" settings below:
 *   - SERVO_MIN_POS / SERVO_MAX_POS: the servo sweeps this whole range in Section 1. Narrow it
 *     if the Limelight's cable gets pulled tight or the camera hits something.
 *   - HIGHER_POSITION_TURNS_RIGHT: if the camera turns AWAY from the tag in Section 2 (the
 *     Driver Station flips between Section 1 and 2 and never reaches 3), flip it.
 *
 * If the camera swings back and forth across the tag instead of settling, lower TRACK_GAIN.
 */
@TeleOp(name = "Limelight Tag Lock", group = "Test")
public class LimelightTagLock extends LinearOpMode {

    // =========================
    // Hardware names (must match the Robot Configuration)
    // =========================
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String CAM_SERVO_NAME = "camServo";

    // =========================
    // AprilTag
    // =========================
    private static final int PIPELINE_INDEX = 0;              // Limelight pipeline set up for AprilTags
    private static final int[] TARGET_TAG_IDS = { 35, 36 };   // the tags this program looks for

    // A result older than this means the Limelight has stopped sending data
    private static final long MAX_RESULT_AGE_MS = 250;

    // =========================
    // Camera servo
    // =========================
    private static final double SERVO_MIN_POS   = 0.0;   // ends of the sweep
    private static final double SERVO_MAX_POS   = 1.0;
    private static final double SERVO_START_POS = 0.5;   // where the servo goes when you press Init

    // Degrees the camera turns when the servo goes from position 0.0 to 1.0.
    // 270 fits a goBILDA 300-degree servo on the SDK's default 600-2400 us range.
    // It turns the speeds below (in degrees) into servo movement, so close is good enough.
    private static final double SERVO_RANGE_DEG = 270.0;

    // true: a higher servo position turns the camera to the right (seen from behind the camera)
    private static final boolean HIGHER_POSITION_TURNS_RIGHT = true;

    // =========================
    // Section 1: search
    // =========================
    private static final double SEARCH_SPEED_DEG_PER_SEC = 30.0;   // sweep speed. Slower = less blur

    // =========================
    // Sections 2 and 3: align and stay locked
    // =========================
    // How hard the servo chases the tag: turn speed (deg/sec) = TRACK_GAIN * tx (deg).
    // Higher follows a moving tag more closely. Too high and the camera swings back and forth
    // across the tag. A slow Limelight pipeline (high resolution, low fps) needs a lower gain.
    private static final double TRACK_GAIN = 5.0;

    // tx this close to zero counts as zero. A servo cannot hold exactly 0.00 degrees.
    // If the servo twitches while locked, or Section 2 never hands over to Section 3, raise this.
    private static final double ON_TARGET_DEG = 0.5;

    // Section 2 hands over to Section 3 once tx has stayed at zero for this long
    private static final double LOCK_CONFIRM_SEC = 0.25;

    // Tag out of sight this long: give up and go back to Section 1. Shorter gaps (blur,
    // something passing in front of the tag) just make the servo wait where it is.
    private static final double LOST_TIMEOUT_SEC = 0.5;

    // Longest time step used for one loop, so one slow loop cannot throw the servo a long way
    private static final double MAX_LOOP_SEC = 0.1;

    private static final int NO_TAG = -1;

    private enum Section { SEARCH, ALIGN, LOCKED }

    private Limelight3A limelight;
    private Servo camServo;

    private Section section = Section.SEARCH;
    private double servoPosition = SERVO_START_POS;   // where the servo is being told to go
    private int searchDirection = +1;                 // +1: servo position going up, -1: going down
    private int followedTagId = NO_TAG;               // the tag (35 or 36) Sections 2 and 3 follow

    // What the camera sees on this loop
    private boolean limelightHasData;
    private LLResultTypes.FiducialResult tag;         // the target tag, or null if it is not in view
    private double lastTx;                            // tx the last time the tag was in view

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final ElapsedTime onTargetTimer = new ElapsedTime();   // how long tx has stayed at zero

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);

        // A servo cannot report where it is, so send it to a known position to start from
        camServo.setPosition(servoPosition);

        // Send telemetry often so the Driver Station keeps up with the camera
        telemetry.setMsTransmissionInterval(11);

        // The Limelight takes a while to boot, so keep asking until it switches pipelines
        while (opModeInInit() && !limelight.pipelineSwitch(PIPELINE_INDEX)) {
            telemetry.addData("Status", "Waiting for the Limelight to respond");
            telemetry.update();
            sleep(500);
        }

        // Without start(), getLatestResult() always returns null
        limelight.start();

        telemetry.addData("Status", "Ready. Press Play to search for AprilTag %s", targetIdsText());
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // Seconds since the last loop, so servo speeds do not depend on how fast the loop runs
            double dt = Math.min(loopTimer.seconds(), MAX_LOOP_SEC);
            loopTimer.reset();

            lookForTag();

            switch (section) {
                case SEARCH: section1Search(dt); break;
                case ALIGN:  section2Align(dt);  break;
                case LOCKED: section3Locked(dt); break;
            }

            camServo.setPosition(servoPosition);
            showTelemetry();
        }

        limelight.stop();
    }

    // =====================================================================================
    // SECTION 1 - SEARCH
    // The servo slowly sweeps the camera until it sees AprilTag 35 or 36.
    // =====================================================================================
    private void section1Search(double dt) {
        if (tag != null) {
            // Found one. Remember which, so Sections 2 and 3 stay on this tag.
            followedTagId = tag.getFiducialId();
            onTargetTimer.reset();
            section = Section.ALIGN;
            return;
        }

        // No camera data: wait here instead of sweeping past a tag the camera cannot report
        if (!limelightHasData) return;

        servoPosition += searchDirection * SEARCH_SPEED_DEG_PER_SEC * dt / SERVO_RANGE_DEG;

        // Turn around at each end of the sweep
        if (servoPosition >= SERVO_MAX_POS) {
            servoPosition = SERVO_MAX_POS;
            searchDirection = -1;
        } else if (servoPosition <= SERVO_MIN_POS) {
            servoPosition = SERVO_MIN_POS;
            searchDirection = +1;
        }
    }

    // =====================================================================================
    // SECTION 2 - ALIGN
    // The servo constantly adjusts its position to make tx = zero degrees.
    // =====================================================================================
    private void section2Align(double dt) {
        if (tagIsLost()) {
            goBackToSearch();
            return;
        }
        if (tag == null) {
            // Out of sight for a moment: hold still, and start the on-target count over
            onTargetTimer.reset();
            return;
        }

        double tx = tag.getTargetXDegrees();
        turnToward(tx, dt);

        // Locked once tx has stayed at zero for LOCK_CONFIRM_SEC without a break
        if (Math.abs(tx) > ON_TARGET_DEG) onTargetTimer.reset();
        if (onTargetTimer.seconds() >= LOCK_CONFIRM_SEC) section = Section.LOCKED;
    }

    // =====================================================================================
    // SECTION 3 - LOCKED
    // The target is locked. If any movement occurs, the servo keeps the camera on target.
    // =====================================================================================
    private void section3Locked(double dt) {
        if (tagIsLost()) {
            goBackToSearch();
            return;
        }
        if (tag == null) return;   // out of sight for a moment: hold still

        // tx only leaves zero when the tag or the robot moves. Turn the camera to follow.
        double tx = tag.getTargetXDegrees();
        if (Math.abs(tx) > ON_TARGET_DEG) turnToward(tx, dt);
    }

    // =====================================================================================
    // Shared by the sections
    // =====================================================================================

    /** Reads the newest Limelight result and picks out the tag the current section wants. */
    private void lookForTag() {
        LLResult result = limelight.getLatestResult();
        limelightHasData = result != null && result.getStaleness() <= MAX_RESULT_AGE_MS;

        tag = limelightHasData ? findTag(result.getFiducialResults()) : null;
        if (tag != null) {
            lastTx = tag.getTargetXDegrees();
            sinceTagSeen.reset();
        }
    }

    /** The wanted tag, or null. If more than one is in view, the biggest (closest) one. */
    private LLResultTypes.FiducialResult findTag(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null) return null;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult candidate : tags) {
            if (!isWanted(candidate.getFiducialId())) continue;
            if (best == null || candidate.getTargetArea() > best.getTargetArea()) best = candidate;
        }
        return best;
    }

    /**
     * Section 1 wants tag 35 or 36. Sections 2 and 3 want only the tag they are following,
     * so the camera does not jump between tags when both are in view.
     */
    private boolean isWanted(int tagId) {
        if (section != Section.SEARCH) return tagId == followedTagId;

        for (int targetId : TARGET_TAG_IDS) {
            if (tagId == targetId) return true;
        }
        return false;
    }

    /**
     * Turns the camera toward the tag. The turn speed is proportional to tx, so the camera
     * moves quickly when the tag is far off center and eases to a stop as tx reaches zero.
     */
    private void turnToward(double txDegrees, double dt) {
        double degreesRight = TRACK_GAIN * txDegrees * dt;
        double positionChange = degreesRight / SERVO_RANGE_DEG;
        if (!HIGHER_POSITION_TURNS_RIGHT) positionChange = -positionChange;

        servoPosition = Range.clip(servoPosition + positionChange, SERVO_MIN_POS, SERVO_MAX_POS);
    }

    private boolean tagIsLost() {
        return tag == null && sinceTagSeen.seconds() > LOST_TIMEOUT_SEC;
    }

    /** Back to Section 1, sweeping first toward the side where the tag was last seen. */
    private void goBackToSearch() {
        boolean lastSeenOnRight = lastTx > 0;
        searchDirection = (lastSeenOnRight == HIGHER_POSITION_TURNS_RIGHT) ? +1 : -1;
        followedTagId = NO_TAG;
        section = Section.SEARCH;
    }

    private void showTelemetry() {
        switch (section) {
            case SEARCH:
                telemetry.addData("Section", "1 - SEARCHING for AprilTag %s", targetIdsText());
                break;
            case ALIGN:
                telemetry.addData("Section", "2 - ALIGNING on AprilTag %d", followedTagId);
                break;
            case LOCKED:
                telemetry.addData("Section", "3 - LOCKED on AprilTag %d", followedTagId);
                break;
        }

        if (!limelightHasData) {
            telemetry.addData("tx", "waiting for Limelight data");
        } else if (tag == null) {
            telemetry.addData("tx", "tag not in view");
        } else {
            telemetry.addData("tx", txText(tag.getTargetXDegrees()));
        }

        boolean atEndOfTravel = servoPosition <= SERVO_MIN_POS || servoPosition >= SERVO_MAX_POS;
        telemetry.addData("Servo position", atEndOfTravel ? "%.3f (end of travel)" : "%.3f", servoPosition);
        telemetry.update();
    }

    private static String txText(double txDegrees) {
        if (Math.abs(txDegrees) < 0.05) return "0.0 deg (centered)";
        return String.format("%.1f deg %s of center", Math.abs(txDegrees), txDegrees > 0 ? "right" : "left");
    }

    /** "35 or 36" */
    private static String targetIdsText() {
        StringBuilder ids = new StringBuilder();
        for (int targetId : TARGET_TAG_IDS) {
            if (ids.length() > 0) ids.append(" or ");
            ids.append(targetId);
        }
        return ids.toString();
    }
}
