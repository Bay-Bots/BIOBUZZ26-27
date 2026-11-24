// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/LimelightTagLockAutoRpm.java
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
 * Limelight Tag Lock + AUTO RPM from tag distance.
 * A copy of LimelightTagLockRpmStepUp where the motor speed is set automatically from how far
 * away the AprilTag is, instead of stepping it up and down by hand.
 *
 * Part A (the camera) is the same as LimelightTagLockRpmStepUp. Part B (the motor) reads the
 * tag's distance from the Limelight and picks the RPM from a table:
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
 * PART B - AUTO RPM FROM TAG DISTANCE for a goBILDA 6000 RPM (5203 Yellow Jacket, 1:1) motor
 *   When you press PLAY the motor spins at DEFAULT_RPM (2500) until the camera sees a tag.
 *   While a tag is in view, the Limelight measures the floor distance to the tag (from the
 *   spot under the tag's center to the spot under the Limelight lens), and the motor speed
 *   comes from a DISTANCE -> RPM table (in between table rows it blends the two nearest rows).
 *   If the tag goes out of view, the motor holds the last speed it had. Speed is closed-loop
 *   (setVelocity), so it holds RPM under load.
 *
 *   Buttons (gamepad1):
 *     - Dpad down:       STOP (motor coasts down)
 *     - A while stopped: restart (back to automatic speed)
 *
 *   WHERE THE TABLE COMES FROM: run the "Shooter Distance Tuner" OpMode (ShooterDistanceTuner).
 *   It has you take test shots from several distances and saves the table on the Robot
 *   Controller. This OpMode loads that saved table when it starts. If there is no saved table,
 *   it uses the built-in TABLE_DISTANCE_IN / TABLE_RPM below, whose RPMs are placeholders.
 *   The Driver Station shows which table is in use.
 *
 *   HOW DISTANCE IS MEASURED (see ShooterRpmTable): from the Limelight's 3D AprilTag data when
 *   the Limelight sends it, otherwise estimated from ty (the tag's angle above the center of
 *   the camera's view) and the camera geometry in ShooterRpmTable: CAMERA_LENS_HEIGHT_IN,
 *   TAG_CENTER_HEIGHT_IN and CAMERA_TILT_DEG. MEASURE THOSE ON YOUR ROBOT. The Driver Station
 *   shows which method is in use. Without a distance, the motor stays at DEFAULT_RPM.
 *
 * Robot Configuration:
 *   - Limelight 3A (plugged into USB, listed with the webcams)        name: "limelight"
 *   - Standard position servo (not continuous rotation) that turns
 *     the Limelight, on a servo port, type Servo                      name: "camServo"
 *   - goBILDA 6000 RPM motor, on a motor port                         name: "spinMotor"
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
@TeleOp(name = "Limelight Tag Lock + Auto RPM", group = "Test")
public class LimelightTagLockAutoRpm extends LinearOpMode {

    // =========================
    // Hardware names (must match the Robot Configuration)
    // =========================
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String CAM_SERVO_NAME = "camServo";
    private static final String MOTOR_NAME     = "spinMotor";

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
    // 270 fits a goBILDA 300-degree servo on the SDK's default 600-2400 us range.
    // It turns the speeds below (in degrees) into servo movement, so close is good enough.
    private static final double SERVO_RANGE_DEG = 270.0;

    // true: a higher servo position turns the camera to the right (seen from behind the camera)
    private static final boolean HIGHER_POSITION_TURNS_RIGHT = true;

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

    private static final int NO_TAG = -1;

    // =========================
    // PART B: Motor
    // =========================
    // goBILDA 6000 RPM Yellow Jacket: 28 encoder ticks per output shaft revolution (1:1 gearbox)
    private static final double TICKS_PER_REV = 28.0;

    private static final double MAX_RPM = 6000.0;

    // Speed used from PLAY until the camera has measured a distance for the first time
    private static final double DEFAULT_RPM = 2500.0;

    // =========================
    // PART B: Built-in DISTANCE -> RPM table. ONLY USED IF THERE IS NO TABLE SAVED BY THE
    // SHOOTER DISTANCE TUNER. THE RPM NUMBERS ARE PLACEHOLDERS.
    // =========================
    // Floor distance to the tag, in inches (closest first). Each distance pairs with the RPM
    // directly below it. Closer than the first row uses the first RPM; farther than the last
    // row uses the last RPM. You can add or remove rows, as long as both lists stay the same
    // length. (The tuner's finish screen shows its table in this form, if you want to paste it.)
    private static final double[] TABLE_DISTANCE_IN = {   24,   36,   48,   60,   72,   96,  120 };
    private static final double[] TABLE_RPM         = { 2500, 2800, 3100, 3400, 3700, 4300, 4900 };

    // Smooths out jitter in the distance reading so the motor speed does not wobble.
    // 1.0 = no smoothing (follows every reading), smaller = smoother but slower to react.
    private static final double DISTANCE_SMOOTHING = 0.3;

    // Readings outside this range are treated as bad data and ignored
    private static final double MIN_VALID_DISTANCE_IN = 2.0;     // 0 means the Limelight sent no 3D data
    private static final double MAX_VALID_DISTANCE_IN = 240.0;

    // Flip to REVERSE if the motor spins the wrong way
    private static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // =========================
    // Hardware
    // =========================
    private Limelight3A limelight;
    private Servo camServo;
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

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final ElapsedTime onTargetTimer = new ElapsedTime();   // how long tx has stayed at zero

    // =========================
    // PART B state
    // =========================
    private enum MotorMode { WAITING_FOR_TAG, AUTO, HOLDING, STOPPED }

    private MotorMode motorMode = MotorMode.WAITING_FOR_TAG;
    private double targetRpm = DEFAULT_RPM;
    private boolean prevA = false;

    private boolean haveDistance = false;    // true once a good distance has been measured
    private double smoothedDistanceIn = 0.0; // filtered distance the RPM is based on
    private double rawDistanceIn = 0.0;      // this loop's reading (0 if none)
    private boolean noDataFromTag = false;   // tag in view but no usable distance from it
    private ShooterRpmTable.DistanceSource distanceSource = ShooterRpmTable.DistanceSource.NONE;
    private double lastTyDeg = 0.0;          // ty of the tag this loop, for the Driver Station

    private ShooterRpmTable table;           // the DISTANCE -> RPM table in use
    private boolean tableFromTuner = false;  // true: loaded from the Shooter Distance Tuner's file

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

        // Use the table saved by the Shooter Distance Tuner if there is one, else the built-in one
        ShooterRpmTable saved = ShooterRpmTable.loadOrNull();
        tableFromTuner = saved != null;
        table = tableFromTuner ? saved : new ShooterRpmTable(TABLE_DISTANCE_IN, TABLE_RPM);

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
        telemetry.addData("Table", tableText());
        telemetry.addLine("Motor speed is automatic from tag distance (2500 RPM until a tag is seen)");
        telemetry.addLine("Dpad down: STOP | A while stopped: restart");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        // Motor starts at DEFAULT_RPM. Don't count an A that was already held when Play was pressed.
        motorMode = MotorMode.WAITING_FOR_TAG;
        targetRpm = DEFAULT_RPM;
        prevA = gamepad1.a;

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

            // PART B: Motor speed from tag distance
            updateMotor();

            showTelemetry();
        }

        motor.setPower(0.0);
        limelight.stop();
    }

    // =====================================================================================
    // PART B - AUTO RPM FROM TAG DISTANCE
    // Speed comes from the DISTANCE -> RPM table. Dpad down: stop, A while stopped: restart.
    // =====================================================================================
    private void updateMotor() {
        // Measure the distance every loop (even while stopped) so the Driver Station shows it
        measureDistance();

        // Edge detection: only react on the moment A goes from released -> pressed
        boolean a = gamepad1.a;
        boolean aPressed = a && !prevA;
        prevA = a;

        if (gamepad1.dpad_down) {
            // Stop wins over everything else
            motorMode = MotorMode.STOPPED;
        } else if (motorMode == MotorMode.STOPPED) {
            // While stopped, A restarts. The speed is picked below, the same as when running.
            if (aPressed) motorMode = haveDistance ? MotorMode.HOLDING : MotorMode.WAITING_FOR_TAG;
        }

        if (motorMode != MotorMode.STOPPED) {
            if (rawDistanceIn > 0.0) {
                // Tag in view with a good distance: speed from the table
                motorMode = MotorMode.AUTO;
                targetRpm = Range.clip(table.rpmFor(smoothedDistanceIn), 0.0, MAX_RPM);
            } else if (haveDistance) {
                // No distance this loop: keep the last speed
                motorMode = MotorMode.HOLDING;
            } else {
                // No distance measured yet since Play
                motorMode = MotorMode.WAITING_FOR_TAG;
                targetRpm = DEFAULT_RPM;
            }
        }

        if (motorMode == MotorMode.STOPPED) {
            // Cut power (coast) rather than commanding 0 velocity, which would actively brake
            motor.setPower(0.0);
        } else {
            motor.setVelocity(rpmToTicksPerSec(targetRpm));
        }
    }

    /**
     * Reads the floor distance to the tag being tracked, in inches, into rawDistanceIn (0 if
     * there is no good reading this loop), and updates the smoothed distance the RPM is based
     * on. Uses ShooterRpmTable's measurement, the same one the Shooter Distance Tuner saves.
     */
    private void measureDistance() {
        rawDistanceIn = 0.0;
        noDataFromTag = false;
        distanceSource = ShooterRpmTable.DistanceSource.NONE;
        if (tag == null) return;

        // 3D data if the Limelight sends it, otherwise the ty estimate (see ShooterRpmTable)
        ShooterRpmTable.DistanceReading reading = ShooterRpmTable.measureFloorDistance(tag);
        lastTyDeg = tag.getTargetYDegrees();
        double distance = reading.inches;
        if (distance < MIN_VALID_DISTANCE_IN || distance > MAX_VALID_DISTANCE_IN) {
            noDataFromTag = true;
            return;
        }

        rawDistanceIn = distance;
        distanceSource = reading.source;
        if (!haveDistance) {
            smoothedDistanceIn = distance;   // first reading: start the filter right there
            haveDistance = true;
        } else {
            smoothedDistanceIn += DISTANCE_SMOOTHING * (distance - smoothedDistanceIn);
        }
    }

    private String tableText() {
        return tableFromTuner
                ? String.format("from Shooter Distance Tuner (%d points)", table.size())
                : "built-in PLACEHOLDER table (run Shooter Distance Tuner to make a real one)";
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

        // PART B: Motor
        double actualRpm = ticksPerSecToRpm(motor.getVelocity());

        telemetry.addLine();
        if (rawDistanceIn > 0.0) {
            telemetry.addData("Floor distance", "%.1f in (smoothed %.1f in)", rawDistanceIn, smoothedDistanceIn);
            telemetry.addData("Measured with", distanceSource == ShooterRpmTable.DistanceSource.TAG_3D
                    ? "Limelight 3D data"
                    : String.format("ty estimate (ty %.1f deg, no 3D data from Limelight)", lastTyDeg));
        } else if (noDataFromTag) {
            telemetry.addData("Floor distance", "CAN'T MEASURE (ty %.1f deg). No 3D data from the Limelight,"
                    + " and the ty estimate is off: check the camera heights/tilt in ShooterRpmTable", lastTyDeg);
        } else if (haveDistance) {
            telemetry.addData("Floor distance", "tag not in view (last %.1f in)", smoothedDistanceIn);
        } else {
            telemetry.addData("Floor distance", "no tag seen yet");
        }
        telemetry.addData("Table", tableText());

        switch (motorMode) {
            case WAITING_FOR_TAG: telemetry.addData("Motor", "DEFAULT %.0f RPM until a tag is seen", DEFAULT_RPM); break;
            case AUTO:            telemetry.addData("Motor", "AUTO from distance"); break;
            case HOLDING:         telemetry.addData("Motor", "HOLDING last speed (no distance right now)"); break;
            case STOPPED:         telemetry.addData("Motor", "STOPPED (press A to restart)"); break;
        }
        telemetry.addData("Target RPM", "%.0f", motorMode == MotorMode.STOPPED ? 0.0 : targetRpm);
        telemetry.addData("Actual RPM", "%.0f", actualRpm);
        telemetry.addLine("Dpad down: STOP | A while stopped: restart");

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
