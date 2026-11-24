// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/LimelightTagLockRpmStepUp.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Limelight Tag Lock + Motor RPM Step Up, combined into one OpMode.
 * (LimelightTagLock.java and OBMotorRpmStepUp.java, running side by side.)
 *
 * The two parts run at the same time and do not affect each other:
 *
 * PART A - LIMELIGHT TAG LOCK (automatic, no buttons)
 *   The Limelight 3A is mounted on a servo that turns it left and right. It finds AprilTag
 *   34, 35, 36 or 37, centers the camera on it, and stays locked on. If more than one is in
 *   view, it picks the biggest (closest) one and sticks with it. It runs in three sections,
 *   and the Driver Station shows which one is running:
 *
 *     SECTION 1 - SEARCH   The servo slowly sweeps the camera back and forth until the Limelight
 *                          sees AprilTag 34, 35, 36 or 37.
 *     SECTION 2 - ALIGN    The servo keeps adjusting its position until tx is zero degrees.
 *                          (tx is the Limelight's horizontal angle from the center of its view
 *                          to the tag. It is positive when the tag is to the right.)
 *     SECTION 3 - LOCKED   The target is locked. If the tag or the robot moves, the servo turns
 *                          the camera to keep tx at zero.
 *
 *   If the tag stays out of sight for LOST_TIMEOUT_SEC in Section 2 or 3, it goes back to
 *   Section 1 and sweeps toward the side where the tag was last seen.
 *
 *   The goBILDA RGB Indicator Light shows what the camera is doing:
 *     red = Section 1, searching     yellow = Section 2, aligning     green = Section 3, locked
 *     off = the Limelight is not sending data
 *   Change the colors with the LIGHT_ settings below. The light is optional: without "rgbLight"
 *   in the Robot Configuration, the program runs the same and the Driver Station says so.
 *
 * PART B - MOTOR RPM STEP UP (gamepad1) for a goBILDA 6000 RPM (5203 Yellow Jacket, 1:1) motor
 *   When you press PLAY the motor spins up to 2500 RPM. Speed is closed-loop (setVelocity),
 *   so it holds RPM under load. One step per press; holding a button does not keep stepping.
 *     - A:               +100 RPM (caps at 6000 RPM)
 *     - B:               -100 RPM (bottoms out at 100 RPM; use Dpad down to fully stop)
 *     - Dpad down:       STOP (motor coasts down)
 *     - A while stopped: restart at 2500 RPM
 *
 * Robot Configuration:
 *   - Limelight 3A (plugged into USB, listed with the webcams)        name: "limelight"
 *   - Standard position servo (not continuous rotation) that turns
 *     the Limelight, on a servo port, type Servo                      name: "camServo"
 *   - goBILDA 6000 RPM motor, on a motor port                         name: "spinMotor"
 *   - goBILDA RGB Indicator Light, on a servo port, type Servo        name: "rgbLight"
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
 * If the motor spins the wrong way, flip MOTOR_DIRECTION.
 */
@TeleOp(name = "Limelight Tag Lock + RPM Step Up", group = "Test")
public class LimelightTagLockRpmStepUp extends LinearOpMode {

    // =========================
    // Hardware names (must match the Robot Configuration)
    // =========================
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String CAM_SERVO_NAME = "camServo";
    private static final String MOTOR_NAME     = "spinMotor";
    private static final String LIGHT_NAME     = "rgbLight";

    // =========================
    // PART A: AprilTag
    // =========================
    private static final int PIPELINE_INDEX = 0;              // Limelight pipeline set up for AprilTags
    private static final int[] TARGET_TAG_IDS = { 34, 35, 36, 37 };   // the tags this program looks for

    // A result older than this means the Limelight has stopped sending data
    private static final long MAX_RESULT_AGE_MS = 250;

    // =========================
    // PART A: Camera servo THESE NEED TO BE MODIFIED SO YOU DONT BREAK THE WHOLE THING
    // =========================
    private static final double SERVO_MIN_POS   = 0.0;   // ends of the sweep
    private static final double SERVO_MAX_POS   = 1.0;
    private static final double SERVO_START_POS = 0.5;   // where the servo goes when you press Init

    // Degrees the camera turns when the servo goes from position 0.0 to 1.0.
    // The servo turns 270 degrees (a goBILDA 300-degree servo on the SDK's default 600-2400 us
    // range), and its 60-tooth gear drives the 176-tooth turret gear: 270 * 60 / 176 = about 92.
    // It turns the speeds below (in degrees) into servo movement, so close is good enough.
    private static final double SERVO_RANGE_DEG = 270.0 * 60.0 / 176.0;

    // true: a higher servo position turns the camera to the right (seen from behind the camera).
    // false on the turret, because the servo gear and the turret gear turn opposite ways.
    private static final boolean HIGHER_POSITION_TURNS_RIGHT = false;

    // =========================
    // PART A, Section 1: search
    // =========================
    private static final double SEARCH_SPEED_DEG_PER_SEC = 30.0;   // sweep speed. Slower = less blur

    // =========================
    // PART A, Sections 2 and 3: align and stay locked
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

    // =========================
    // PART A: RGB indicator light, the color for each section (any color from RgbIndicatorLight)
    // =========================
    private static final double LIGHT_NO_DATA   = RgbIndicatorLight.OFF;     // Limelight not sending data
    private static final double LIGHT_SEARCHING = RgbIndicatorLight.RED;     // Section 1: looking for a tag
    private static final double LIGHT_ALIGNING  = RgbIndicatorLight.YELLOW;  // Section 2: centering on the tag
    private static final double LIGHT_LOCKED    = RgbIndicatorLight.GREEN;   // Section 3: locked on the tag

    private static final int NO_TAG = -1;

    // =========================
    // PART B: Motor
    // =========================
    // goBILDA 6000 RPM Yellow Jacket: 28 encoder ticks per output shaft revolution (1:1 gearbox)
    private static final double TICKS_PER_REV = 28.0;

    private static final double START_RPM = 2500.0;
    private static final double STEP_RPM  = 100.0;
    private static final double MIN_RPM   = 100.0;  // lowest B can step down to while running
    private static final double MAX_RPM   = 6000.0;

    // Flip to REVERSE if the motor spins the wrong way
    private static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // =========================
    // Hardware
    // =========================
    private Limelight3A limelight;
    private Servo camServo;
    private Servo light;                              // RGB indicator light, or null if not configured
    private DcMotorEx motor;

    // =========================
    // PART A state
    // =========================
    private enum Section { SEARCH, ALIGN, LOCKED }

    private Section section = Section.SEARCH;
    private double servoPosition = SERVO_START_POS;   // where the servo is being told to go
    private int searchDirection = +1;                 // +1: servo position going up, -1: going down
    private int followedTagId = NO_TAG;               // the tag (34-37) Sections 2 and 3 follow

    // What the camera sees on this loop
    private boolean limelightHasData;
    private LLResultTypes.FiducialResult tag;         // the target tag, or null if it is not in view
    private double lastTx;                            // tx the last time the tag was in view

    private double lightShowing = RgbIndicatorLight.OFF;   // the color last sent to the light

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final ElapsedTime onTargetTimer = new ElapsedTime();   // how long tx has stayed at zero

    // =========================
    // PART B state
    // =========================
    private double targetRpm = START_RPM;
    private boolean motorStopped = false;
    private boolean prevA = false;
    private boolean prevB = false;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        // Motor setup. FLOAT so it coasts to a stop instead of slamming the brakes at high RPM.
        motor.setDirection(MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0.0);

        // A servo cannot report where it is, so send it to a known position to start from
        camServo.setPosition(servoPosition);

        // The light is optional: tryGet returns null, instead of stopping with an error, if there
        // is no "rgbLight" in the Robot Configuration. It stays off until Play.
        light = hardwareMap.tryGet(Servo.class, LIGHT_NAME);
        setLight(RgbIndicatorLight.OFF);

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
        telemetry.addData("Light", lightText());
        telemetry.addLine("Motor starts at 2500 RPM on Play");
        telemetry.addLine("A: +100 | B: -100 | Dpad down: STOP");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        // Motor starts at START_RPM. Don't count a button that was already held when Play was pressed.
        targetRpm = START_RPM;
        motorStopped = false;
        prevA = gamepad1.a;
        prevB = gamepad1.b;

        while (opModeIsActive()) {
            // Seconds since the last loop, so servo speeds do not depend on how fast the loop runs
            double dt = Math.min(loopTimer.seconds(), MAX_LOOP_SEC);
            loopTimer.reset();

            // PART A: Limelight tag lock
            lookForTag();

            switch (section) {
                case SEARCH: section1Search(dt); break;
                case ALIGN:  section2Align(dt);  break;
                case LOCKED: section3Locked(dt); break;
            }

            camServo.setPosition(servoPosition);
            setLight(lightColor());

            // PART B: Motor RPM step up
            updateMotor();

            showTelemetry();
        }

        motor.setPower(0.0);
        setLight(RgbIndicatorLight.OFF);
        limelight.stop();
    }

    // =====================================================================================
    // PART B - MOTOR RPM STEP UP
    // A: +100 RPM, B: -100 RPM, Dpad down: stop, A while stopped: restart at 2500 RPM.
    // =====================================================================================
    private void updateMotor() {
        // Edge detection: only step on the moment a button goes from released -> pressed
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        prevA = a;
        prevB = b;

        if (gamepad1.dpad_down) {
            // Stop wins over everything else
            motorStopped = true;
        } else if (motorStopped) {
            // While stopped, A restarts at the starting speed; B does nothing
            if (aPressed) {
                targetRpm = START_RPM;
                motorStopped = false;
            }
        } else {
            if (aPressed) targetRpm += STEP_RPM;
            if (bPressed) targetRpm -= STEP_RPM;
            targetRpm = Range.clip(targetRpm, MIN_RPM, MAX_RPM);
        }

        if (motorStopped) {
            // Cut power (coast) rather than commanding 0 velocity, which would actively brake
            motor.setPower(0.0);
        } else {
            motor.setVelocity(rpmToTicksPerSec(targetRpm));
        }
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }

    // =====================================================================================
    // PART A, SECTION 1 - SEARCH
    // The servo slowly sweeps the camera until it sees AprilTag 34, 35, 36 or 37.
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
    // PART A, SECTION 2 - ALIGN
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
    // PART A, SECTION 3 - LOCKED
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
    // PART A - shared by the sections
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
     * Section 1 wants tag 34, 35, 36 or 37. Sections 2 and 3 want only the tag they are following,
     * so the camera does not jump between tags when more than one is in view.
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

    /** The light's color for what the camera is doing right now (see the LIGHT_ settings). */
    private double lightColor() {
        if (!limelightHasData) return LIGHT_NO_DATA;
        switch (section) {
            case ALIGN:  return LIGHT_ALIGNING;
            case LOCKED: return LIGHT_LOCKED;
            default:     return LIGHT_SEARCHING;
        }
    }

    /** Sends a color to the RGB indicator light, if there is one. */
    private void setLight(double color) {
        lightShowing = color;
        if (light != null) light.setPosition(color);
    }

    /** The light's color, or why there is no light, for the Driver Station. */
    private String lightText() {
        if (light == null) return "none (no Servo named \"" + LIGHT_NAME + "\" in the Robot Configuration)";
        return RgbIndicatorLight.name(lightShowing);
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

    // =====================================================================================
    // Telemetry (both parts)
    // =====================================================================================
    private void showTelemetry() {
        // PART A: Limelight
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
        telemetry.addData("Light", lightText());

        // PART B: Motor
        double actualRpm = ticksPerSecToRpm(motor.getVelocity());

        telemetry.addLine();
        telemetry.addData("Motor", motorStopped ? "STOPPED (press A to restart at 2500)" : "RUNNING");
        telemetry.addData("Target RPM", "%.0f", motorStopped ? 0.0 : targetRpm);
        telemetry.addData("Actual RPM", "%.0f", actualRpm);
        if (!motorStopped && targetRpm >= MAX_RPM) telemetry.addLine("At max (6000 RPM)");
        if (!motorStopped && targetRpm <= MIN_RPM) telemetry.addLine("At min (100 RPM) - Dpad down to stop");
        telemetry.addLine("A: +100 | B: -100 | Dpad down: STOP");

        telemetry.update();
    }

    private static String txText(double txDegrees) {
        if (Math.abs(txDegrees) < 0.05) return "0.0 deg (centered)";
        return String.format("%.1f deg %s of center", Math.abs(txDegrees), txDegrees > 0 ? "right" : "left");
    }

    /** "34, 35, 36 or 37" */
    private static String targetIdsText() {
        StringBuilder ids = new StringBuilder();
        for (int i = 0; i < TARGET_TAG_IDS.length; i++) {
            if (i > 0) ids.append(i == TARGET_TAG_IDS.length - 1 ? " or " : ", ");
            ids.append(TARGET_TAG_IDS[i]);
        }
        return ids.toString();
    }
}
